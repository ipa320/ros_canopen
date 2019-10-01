// Copyright (c) 2016-2019, Fraunhofer, Mathias Lüdtke, AutonomouStuff
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <gtest/gtest.h>

#include <socketcan_interface/dispatcher.hpp>

class Counter {
public:
    size_t counter_;
    Counter() : counter_(0) {}
    void count(const can::Frame &msg) {
        ++counter_;
    }
};

TEST(DispatcherTest, testFilteredDispatcher)
{
    can::FilteredDispatcher<unsigned int, can::CommInterface::FrameListener> dispatcher;
    Counter counter1;
    Counter counter2;
    std::vector<can::CommInterface::FrameListenerConstSharedPtr> listeners;
    const size_t max_id = (1<<11);
    for(size_t i=0; i < max_id; i+=2) {
        listeners.push_back(dispatcher.createListener(can::MsgHeader(i), can::CommInterface::FrameDelegate(&counter1, &Counter::count)));
        listeners.push_back(dispatcher.createListener(can::MsgHeader(i+1), can::CommInterface::FrameDelegate(&counter2, &Counter::count)));
    }

    boost::chrono::steady_clock::time_point start = boost::chrono::steady_clock::now();
    const size_t num = 1000 * max_id;
    for(size_t i=0; i < num; ++i) {
        dispatcher.dispatch(can::Frame(can::MsgHeader(i%max_id)));
    }
    boost::chrono::steady_clock::time_point now = boost::chrono::steady_clock::now();
    double diff = boost::chrono::duration_cast<boost::chrono::duration<double> >(now-start).count();

    EXPECT_EQ(num, counter1.counter_+ counter2.counter_);
    EXPECT_EQ(counter1.counter_, counter2.counter_);
    std::cout << std::fixed << diff << "\t" <<  num << "\t" << num / diff << std::endl;

}

TEST(DispatcherTest, testSimpleDispatcher)
{
    can::SimpleDispatcher<can::CommInterface::FrameListener> dispatcher;
    Counter counter;
    can::CommInterface::FrameListenerConstSharedPtr listener = dispatcher.createListener(can::CommInterface::FrameDelegate(&counter, &Counter::count));

    boost::chrono::steady_clock::time_point start = boost::chrono::steady_clock::now();
    const size_t max_id = (1<<11);
    const size_t num = 1000*max_id;
    for(size_t i=0; i < num; ++i) {
        dispatcher.dispatch(can::Frame(can::MsgHeader(i%max_id)));
    }
    boost::chrono::steady_clock::time_point now = boost::chrono::steady_clock::now();
    double diff = boost::chrono::duration_cast<boost::chrono::duration<double> >(now-start).count();

    EXPECT_EQ(num, counter.counter_);
    std::cout << std::fixed << diff << "\t" <<  num << "\t" << num / diff << std::endl;
}

TEST(DispatcherTest, testDelegateOnly)
{
    Counter counter;
    can::CommInterface::FrameDelegate delegate(&counter, &Counter::count);

    boost::chrono::steady_clock::time_point start = boost::chrono::steady_clock::now();
    const size_t max_id = (1<<11);
    const size_t num = 10000*max_id;
    for(size_t i=0; i < num; ++i) {
        delegate(can::Frame(can::MsgHeader(i%max_id)));
    }
    boost::chrono::steady_clock::time_point now = boost::chrono::steady_clock::now();
    double diff = boost::chrono::duration_cast<boost::chrono::duration<double> >(now-start).count();

    EXPECT_EQ(num, counter.counter_);
    std::cout << std::fixed << diff << "\t" <<  num << "\t" << num / diff << std::endl;

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
