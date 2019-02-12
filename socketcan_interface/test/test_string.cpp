// Bring in my package's API, which is what I'm testing
#include <socketcan_interface/string.h>

// Bring in gtest
#include <gtest/gtest.h>


TEST(StringTest, stringconversion)
{
  const std::string s1("123#1234567812345678");
  can::Frame f1 = can::toframe(s1);
  EXPECT_EQ(s1, can::tostring(f1, true));

  const std::string s2("1337#1234567812345678");
  can::Frame f2 = can::toframe(s2);
  EXPECT_FALSE(f2.isValid());

  const std::string s3("80001337#1234567812345678");
  const std::string s4("00001337#1234567812345678");

  can::Frame f3 = can::toframe(s3);
  EXPECT_EQ(f3.fullid(), 0x80001337);
  EXPECT_TRUE(f3.isValid());
  EXPECT_TRUE(f3.is_extended);
  EXPECT_EQ(s4, can::tostring(f3, true)); // 8000 is converted to 0000

  can::Frame f4 = can::toframe(s4);
  EXPECT_EQ(f4.fullid(), 0x80001337);
  EXPECT_TRUE(f4.isValid());
  EXPECT_TRUE(f4.is_extended);
  EXPECT_EQ(s4, can::tostring(f4, true));

  const std::string s5("20001337#1234567812345678");
  can::Frame f5 = can::toframe(s5);
  EXPECT_EQ(f5.fullid(), 0xA0001337);
  EXPECT_TRUE(f5.isValid());
  EXPECT_TRUE(f5.is_error);
  EXPECT_EQ(s5, can::tostring(f5, true));

  const std::string s6("40001337#1234567812345678");
  can::Frame f6 = can::toframe(s6);
  EXPECT_EQ(f6.fullid(), 0xC0001337);
  EXPECT_TRUE(f6.isValid());
  EXPECT_TRUE(f6.is_rtr);
  EXPECT_EQ(s6, can::tostring(f6, true));

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
