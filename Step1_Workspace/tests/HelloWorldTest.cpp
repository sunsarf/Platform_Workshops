#include "HelloWorld.hpp"
#include <gtest/gtest.h>

TEST(HelloWorldTest, EnglishOutput) {
    HelloWorld hello;
    testing::internal::CaptureStdout();
    hello.inEnglish();
    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(output, "Hello World\n");
}

TEST(HelloWorldTest, SpanishOutput) {
    HelloWorld hello;
    testing::internal::CaptureStdout();
    hello.inSpanish();
    std::string output = testing::internal::GetCapturedStdout();
    EXPECT_EQ(output, "Hola Mundo\n");
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}