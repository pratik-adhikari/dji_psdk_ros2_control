#include <gtest/gtest.h>
#include "geogimbal/coordinate_transformer.hpp"

class CoordinateTransformerTest : public ::testing::Test {
protected:
    CoordinateTransformer transformer_;
};

TEST_F(CoordinateTransformerTest, TestSetupUTMValid)
{
    EXPECT_NO_THROW(
        transformer_.setup(FrameType::UTM, 15, true, 0.0, 0.0, 0.0)
    );
}

TEST_F(CoordinateTransformerTest, TestWgs84ToUtm)
{
    transformer_.setup(FrameType::UTM, 15, true, 0.0, 0.0, 0.0);

    // WGS84 in degrees, interpret as (longitude=-93, latitude=42)
    Coordinate input;
    input.longitude = -93.0;  // deg
    input.latitude  =  42.0;  // deg
    input.altitude  = 300.0;  // m

    Coordinate output;
    transformer_.wgs84ToFrame(input, output);

    // Typical result for zone 15N:
    // Easting ~ 500000, Northing ~ 4649776, alt ~ 300
    // We'll allow some range
    EXPECT_NEAR(output.longitude, 500000.0, 3000.0); 
    EXPECT_NEAR(output.latitude,  4649000.0, 3000.0);
    EXPECT_DOUBLE_EQ(output.altitude, 300.0);
}

TEST_F(CoordinateTransformerTest, TestUtmToWgs84)
{
    transformer_.setup(FrameType::UTM, 15, true, 0.0, 0.0, 0.0);

    // Easting=500000, Northing=4649776 => roughly lat=42, lon=-93
    Coordinate input;
    input.longitude = 500000.0;  // easting
    input.latitude  = 4649776.0; // northing
    input.altitude  = 250.0;

    Coordinate output;
    transformer_.frameToWgs84(input, output);

    // Expect ~ (longitude=-93, latitude=42)
    EXPECT_NEAR(output.longitude, -93.0, 1.5);
    EXPECT_NEAR(output.latitude,   42.0, 1.5);
    EXPECT_DOUBLE_EQ(output.altitude, 250.0);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
