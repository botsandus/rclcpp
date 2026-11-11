// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <chrono>
#include <future>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "test_msgs/msg/empty.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/allocator/allocator_common.hpp"

// For demonstration purposes only, not necessary for allocator_traits
static uint32_t num_allocs = 0;
static uint32_t num_deallocs = 0;
// A very simple custom allocator. Counts calls to allocate and deallocate.
template<typename T>
struct MyAllocator
{
public:
  using value_type = T;
  using size_type = std::size_t;
  using pointer = T *;
  using const_pointer = const T *;
  using difference_type = typename std::pointer_traits<pointer>::difference_type;

  MyAllocator() noexcept
  {
  }

  ~MyAllocator() noexcept {}

  template<typename U>
  MyAllocator(const MyAllocator<U> &) noexcept
  {
  }

  T * allocate(size_t size, const void * = 0)
  {
    if (size == 0) {
      return nullptr;
    }
    num_allocs++;
    return static_cast<T *>(std::malloc(size * sizeof(T)));
  }

  void deallocate(T * ptr, [[maybe_unused]] size_t size)
  {
    if (!ptr) {
      return;
    }
    num_deallocs++;
    std::free(ptr);
  }

  template<typename U>
  struct rebind
  {
    typedef MyAllocator<U> other;
  };

  rcl_allocator_t get_rcl_allocator()
  {
    return rcl_get_default_allocator();
  }
};

// Explicit specialization for void
template<>
struct MyAllocator<void>
{
public:
  using value_type = void;
  using pointer = void *;
  using const_pointer = const void *;

  MyAllocator() noexcept
  {
  }

  ~MyAllocator() noexcept {}

  template<typename U>
  MyAllocator(const MyAllocator<U> &) noexcept
  {
  }

  template<typename U>
  struct rebind
  {
    typedef MyAllocator<U> other;
  };

  rcl_allocator_t get_rcl_allocator()
  {
    return rcl_get_default_allocator();
  }
};

template<typename T, typename U>
constexpr bool operator==(
  const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept
{
  return true;
}

template<typename T, typename U>
constexpr bool operator!=(
  const MyAllocator<T> &,
  const MyAllocator<U> &) noexcept
{
  return false;
}

template<
  typename PublishedMessageAllocatorT,
  typename PublisherAllocatorT,
  typename SubscribedMessageAllocatorT,
  typename SubscriptionAllocatorT,
  typename MessageMemoryStrategyAllocatorT,
  typename MemoryStrategyAllocatorT,
  typename ExpectedExceptionT
>
void
do_custom_allocator_test(
  PublishedMessageAllocatorT published_message_allocator,
  PublisherAllocatorT publisher_allocator,
  SubscribedMessageAllocatorT /* subscribed_message_allocator */,  // intentionally unused
  SubscriptionAllocatorT subscription_allocator,
  MessageMemoryStrategyAllocatorT message_memory_strategy,
  MemoryStrategyAllocatorT memory_strategy_allocator)
{
  using PublishedMessageAllocTraits =
    rclcpp::allocator::AllocRebind<test_msgs::msg::Empty, PublishedMessageAllocatorT>;
  using PublishedMessageAlloc = typename PublishedMessageAllocTraits::allocator_type;
  using PublishedMessageDeleter =
    rclcpp::allocator::Deleter<PublishedMessageAlloc, test_msgs::msg::Empty>;

  using SubscribedMessageAllocTraits =
    rclcpp::allocator::AllocRebind<test_msgs::msg::Empty, SubscribedMessageAllocatorT>;
  using SubscribedMessageAlloc = typename SubscribedMessageAllocTraits::allocator_type;
  using SubscribedMessageDeleter =
    rclcpp::allocator::Deleter<SubscribedMessageAlloc, test_msgs::msg::Empty>;

  // init and node
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "custom_allocator_test",
    rclcpp::NodeOptions().context(context).use_intra_process_comms(true));

  // publisher
  auto shared_publisher_allocator = std::make_shared<PublisherAllocatorT>(publisher_allocator);
  rclcpp::PublisherOptionsWithAllocator<PublisherAllocatorT> publisher_options;
  publisher_options.allocator = shared_publisher_allocator;
  auto publisher =
    node->create_publisher<test_msgs::msg::Empty>("custom_allocator_test", 10, publisher_options);

  // callback for subscription
  uint32_t counter = 0;
  std::promise<std::unique_ptr<test_msgs::msg::Empty, SubscribedMessageDeleter>> received_message;
  auto received_message_future = received_message.get_future();
  auto callback =
    [&counter, &received_message](
    std::unique_ptr<test_msgs::msg::Empty, SubscribedMessageDeleter> msg)
    {
      ++counter;
      received_message.set_value(std::move(msg));
    };

  // subscription
  auto shared_subscription_allocator =
    std::make_shared<SubscriptionAllocatorT>(subscription_allocator);
  rclcpp::SubscriptionOptionsWithAllocator<SubscriptionAllocatorT> subscription_options;
  subscription_options.allocator = shared_subscription_allocator;
  auto shared_message_strategy_allocator =
    std::make_shared<MessageMemoryStrategyAllocatorT>(message_memory_strategy);
  auto msg_mem_strat = std::make_shared<
    rclcpp::message_memory_strategy::MessageMemoryStrategy<
      test_msgs::msg::Empty,
      MessageMemoryStrategyAllocatorT
    >
    >(shared_message_strategy_allocator);
  using CallbackMessageT =
    typename rclcpp::subscription_traits::has_message_type<decltype(callback)>::type;
  auto subscriber = node->create_subscription<
    test_msgs::msg::Empty,
    decltype(callback),
    SubscriptionAllocatorT,
    rclcpp::Subscription<CallbackMessageT, SubscriptionAllocatorT>,
    rclcpp::message_memory_strategy::MessageMemoryStrategy<
      CallbackMessageT,
      MessageMemoryStrategyAllocatorT
    >
    >(
    "custom_allocator_test",
    10,
    std::forward<decltype(callback)>(callback),
    subscription_options,
    msg_mem_strat);

  // executor memory strategy
  auto shared_memory_strategy_allocator = std::make_shared<MemoryStrategyAllocatorT>(
    memory_strategy_allocator);

  // executor
  rclcpp::ExecutorOptions options;
  options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(options);

  executor.add_node(node);

  // rebind message_allocator to ensure correct type
  PublishedMessageDeleter message_deleter;
  PublishedMessageAlloc rebound_message_allocator = published_message_allocator;
  rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &rebound_message_allocator);

  // allocate a message
  auto ptr = PublishedMessageAllocTraits::allocate(rebound_message_allocator, 1);
  PublishedMessageAllocTraits::construct(rebound_message_allocator, ptr);
  std::unique_ptr<test_msgs::msg::Empty, PublishedMessageDeleter> msg(ptr, message_deleter);

  // publisher and receive
  if constexpr (std::is_same_v<ExpectedExceptionT, void>) {
    // no exception expected
    EXPECT_NO_THROW(
    {
      publisher->publish(std::move(msg));
      executor.spin_until_future_complete(received_message_future, std::chrono::seconds(10));
    });
    EXPECT_EQ(ptr, received_message_future.get().get());
    EXPECT_EQ(1u, counter);
  } else {
    // exception expected
    EXPECT_THROW(
    {
      publisher->publish(std::move(msg));
      executor.spin_until_future_complete(received_message_future, std::chrono::seconds(10));
    }, ExpectedExceptionT);
  }
}

/*
   This tests the case where a custom allocator is used correctly, i.e. the same
   custom allocator on both sides.
 */
TEST(TestIntraProcessManagerWithAllocators, custom_allocator) {
  using PublishedMessageAllocatorT = std::allocator<void>;
  using PublisherAllocatorT = std::allocator<void>;
  using SubscribedMessageAllocatorT = std::allocator<void>;
  using SubscriptionAllocatorT = std::allocator<void>;
  using MessageMemoryStrategyAllocatorT = std::allocator<void>;
  using MemoryStrategyAllocatorT = std::allocator<void>;
  auto allocator = std::allocator<void>();
  do_custom_allocator_test<
    PublishedMessageAllocatorT,
    PublisherAllocatorT,
    SubscribedMessageAllocatorT,
    SubscriptionAllocatorT,
    MessageMemoryStrategyAllocatorT,
    MemoryStrategyAllocatorT,
    void  // no exception expected
  >(allocator, allocator, allocator, allocator, allocator, allocator);
}

/*
   This tests the case where a custom allocator is used incorrectly, i.e. different
   custom allocators on both sides.
 */
TEST(TestIntraProcessManagerWithAllocators, custom_allocator_wrong) {
  // explicitly use a different allocator here to provoke a failure
  using PublishedMessageAllocatorT = std::allocator<void>;
  using PublisherAllocatorT = std::allocator<void>;
  using SubscribedMessageAllocatorT = MyAllocator<void>;
  using SubscriptionAllocatorT = MyAllocator<void>;
  using MessageMemoryStrategyAllocatorT = MyAllocator<void>;
  using MemoryStrategyAllocatorT = std::allocator<void>;
  auto allocator = std::allocator<void>();
  auto my_allocator = MyAllocator<void>();
  do_custom_allocator_test<
    PublishedMessageAllocatorT,
    PublisherAllocatorT,
    SubscribedMessageAllocatorT,
    SubscriptionAllocatorT,
    MessageMemoryStrategyAllocatorT,
    MemoryStrategyAllocatorT,
    std::runtime_error  // expected exception
  >(allocator, allocator, my_allocator, my_allocator, my_allocator, allocator);
}

struct InterProcessMixResult
{
  uint32_t allocations_during_publish = 0;
  size_t intra_process_received = 0;
  bool inter_process_received = false;
  bool owner_received_published_message = false;
};

/*
   Publish one message to `num_owning` intra-process subscriptions taking a unique_ptr,
   `num_shared` intra-process subscriptions taking a shared_ptr, and one subscription
   with intra-process disabled (so an inter-process publish is needed).
   Messages are allocated with MyAllocator, so `allocations_during_publish` counts the
   message copies made by rclcpp while publishing.
 */
void
publish_to_intra_and_inter_process_subscriptions(
  size_t num_owning, size_t num_shared, InterProcessMixResult & result)
{
  using AllocatorT = MyAllocator<void>;
  using MessageAllocTraits = rclcpp::allocator::AllocRebind<test_msgs::msg::Empty, AllocatorT>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, test_msgs::msg::Empty>;
  using MessageUniquePtr = std::unique_ptr<test_msgs::msg::Empty, MessageDeleter>;
  using MessageMemoryStrategyT =
    rclcpp::message_memory_strategy::MessageMemoryStrategy<test_msgs::msg::Empty, AllocatorT>;
  const std::string topic = "inter_process_mix_test";

  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "inter_process_mix_test",
    rclcpp::NodeOptions().context(context).use_intra_process_comms(true));

  rclcpp::PublisherOptionsWithAllocator<AllocatorT> publisher_options;
  publisher_options.allocator = std::make_shared<AllocatorT>();
  auto publisher =
    node->create_publisher<test_msgs::msg::Empty>(topic, 10, publisher_options);

  const test_msgs::msg::Empty * published_ptr = nullptr;

  rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> subscription_options;
  subscription_options.allocator = std::make_shared<AllocatorT>();
  auto msg_mem_strat = std::make_shared<MessageMemoryStrategyT>(std::make_shared<AllocatorT>());

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  for (size_t i = 0; i < num_owning; ++i) {
    auto callback = [&result, &published_ptr](MessageUniquePtr msg) {
        ++result.intra_process_received;
        if (msg.get() == published_ptr) {
          result.owner_received_published_message = true;
        }
      };
    subscriptions.push_back(
      node->create_subscription<
        test_msgs::msg::Empty, decltype(callback), AllocatorT,
        rclcpp::Subscription<test_msgs::msg::Empty, AllocatorT>, MessageMemoryStrategyT>(
        topic, 10, std::move(callback), subscription_options, msg_mem_strat));
  }
  for (size_t i = 0; i < num_shared; ++i) {
    auto callback = [&result](std::shared_ptr<const test_msgs::msg::Empty>) {
        ++result.intra_process_received;
      };
    subscriptions.push_back(
      node->create_subscription<
        test_msgs::msg::Empty, decltype(callback), AllocatorT,
        rclcpp::Subscription<test_msgs::msg::Empty, AllocatorT>, MessageMemoryStrategyT>(
        topic, 10, std::move(callback), subscription_options, msg_mem_strat));
  }

  rclcpp::SubscriptionOptions inter_process_options;
  inter_process_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  subscriptions.push_back(
    node->create_subscription<test_msgs::msg::Empty>(
      topic, 10,
      [&result](std::shared_ptr<const test_msgs::msg::Empty>) {
        result.inter_process_received = true;
      }, inter_process_options));

  // Wait for the middleware to match the inter-process subscription.
  const size_t expected_intra = num_owning + num_shared;
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (publisher->get_subscription_count() < expected_intra + 1 &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(expected_intra + 1, publisher->get_subscription_count());
  ASSERT_EQ(expected_intra, publisher->get_intra_process_subscription_count());

  rclcpp::ExecutorOptions options;
  options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(options);
  executor.add_node(node);

  MessageAlloc message_allocator;
  MessageDeleter message_deleter;
  rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &message_allocator);
  auto ptr = MessageAllocTraits::allocate(message_allocator, 1);
  MessageAllocTraits::construct(message_allocator, ptr);
  MessageUniquePtr msg(ptr, message_deleter);
  published_ptr = ptr;

  num_allocs = 0;
  publisher->publish(std::move(msg));
  result.allocations_during_publish = num_allocs;

  const auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while ((result.intra_process_received < expected_intra || !result.inter_process_received) &&
    std::chrono::steady_clock::now() < receive_deadline)
  {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(expected_intra, result.intra_process_received);
  EXPECT_TRUE(result.inter_process_received);

  context->shutdown("test finished");
}

/*
   With only an owning intra-process subscription, the inter-process publish must not
   require a copy of the message: the owning subscription receives the published message.
 */
TEST(TestIntraProcessManagerWithAllocators, inter_process_mix_one_owning_no_copy) {
  InterProcessMixResult result;
  publish_to_intra_and_inter_process_subscriptions(1, 0, result);
  EXPECT_EQ(0u, result.allocations_during_publish);
  EXPECT_TRUE(result.owner_received_published_message);
}

/*
   With N owning intra-process subscriptions, only the N-1 copies needed for ownership
   are made.
 */
TEST(TestIntraProcessManagerWithAllocators, inter_process_mix_two_owning_one_copy) {
  InterProcessMixResult result;
  publish_to_intra_and_inter_process_subscriptions(2, 0, result);
  EXPECT_EQ(1u, result.allocations_during_publish);
  EXPECT_TRUE(result.owner_received_published_message);
}

/*
   With owning and shared intra-process subscriptions, one shared copy is required,
   and it is reused for the inter-process publish.
 */
TEST(TestIntraProcessManagerWithAllocators, inter_process_mix_owning_and_shared_one_copy) {
  InterProcessMixResult result;
  publish_to_intra_and_inter_process_subscriptions(1, 1, result);
  EXPECT_EQ(1u, result.allocations_during_publish);
  EXPECT_TRUE(result.owner_received_published_message);
}

/*
   With only shared intra-process subscriptions, the message is promoted without a copy.
 */
TEST(TestIntraProcessManagerWithAllocators, inter_process_mix_shared_only_no_copy) {
  InterProcessMixResult result;
  publish_to_intra_and_inter_process_subscriptions(0, 1, result);
  EXPECT_EQ(0u, result.allocations_during_publish);
}
