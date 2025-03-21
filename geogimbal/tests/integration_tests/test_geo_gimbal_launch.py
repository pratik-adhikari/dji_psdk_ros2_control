# test_geo_gimbal_launch.py
import os
import pytest
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

@pytest.mark.launch_test
def generate_test_description():
    # Instead of generate_test_description_from_launch_file, do it manually:
    ld = launch.LaunchDescription()

    test_launch = os.path.join(
        os.path.dirname(__file__),
        'geo_gimbal_integration_test.launch.py'
    )
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(test_launch),
        launch_arguments={}
    ))

    # Mark the point at which tests can start
    ld.add_action(launch_testing.actions.ReadyToTest())

    return ld, {}

# Then define test cases
def test_fake_check():
    # Put any code checks or an example that node(s) are alive
    assert True
