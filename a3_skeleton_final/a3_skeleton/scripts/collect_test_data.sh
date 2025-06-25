#!/bin/bash

# Enhanced Test Data Collection Script for a3_skeleton
# Integrates with your existing test_mission.sh (Goals in this script match local test_mission.sh script)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_DATA_DIR="${SCRIPT_DIR}/../test_data"
PACKAGE_NAME="a3_skeleton"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
echo_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
echo_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
echo_error() { echo -e "${RED}[ERROR]${NC} $1"; }

mkdir -p "${TEST_DATA_DIR}"

# Function to publish goals
publish_test_goals() {
    local test_type="$1"
    
    echo_info "Publishing goals for ${test_type} test..."
    
    case "$test_type" in
        "gridmap")
            # Random goals, no Z because of altitude control
            ros2 topic pub --once /mission/goals geometry_msgs/msg/PoseArray '{
                "header": {"frame_id": "world"},
                "poses": [
                    {"position": {"x": -10.0, "y": 0.0}, "orientation": {"w": 1.0}},
                    {"position": {"x": -10.0, "y": 5.0}, "orientation": {"w": 1.0}},
                    {"position": {"x": -5.0, "y": -3.0}, "orientation": {"w": 1.0}},
                    {"position": {"x": 4.0, "y": 4.0}, "orientation": {"w": 1.0}}
                ]
            }'
            ;;
        "traversability")
            # Mixed terrain difficulty goals, Z > 2 to override altitude control
            ros2 topic pub --once /mission/goals geometry_msgs/msg/PoseArray '{
                "header": {"frame_id": "world"},
                "poses": [
                    {"position": {"x": 2.0, "y": 2.0, "z": 0.0}, "orientation": {"w": 1.0}},
                    {"position": {"x": 5.0, "y": 3.0, "z": 2.0}, "orientation": {"w": 1.0}},
                    {"position": {"x": -3.0, "y": 4.0, "z": 0.0}, "orientation": {"w": 1.0}},
                    {"position": {"x": 1.0, "y": -2.0, "z": 3.0}, "orientation": {"w": 1.0}}
                ]
            }'
            ;;
        "tsp")
            # Multiple goals for TSP optimisation Matching goals from gridmap test for map info 
            ros2 topic pub --once /mission/goals geometry_msgs/msg/PoseArray '{
                "header": {"frame_id": "world"},
                "poses": [
                    {"position": {"x": -10.0, "y": 0.0}, "orientation": {"w": 1.0}},
                    {"position": {"x": -10.0, "y": 5.0}, "orientation": {"w": 1.0}},
                    {"position": {"x": -5.0, "y": -3.0}, "orientation": {"w": 1.0}},
                    {"position": {"x": 4.0, "y": 4.0}, "orientation": {"w": 1.0}},
                    {"position": {"x": 0.0, "y": 0.0}, "orientation": {"w": 1.0}}
                ]
            }'
            ;;
    esac
    
    # Give time for goal processing
    sleep 2
}

# Function to record test data
record_test_data() {
    local test_name="$1"
    local duration="$2"
    local bag_name="${TEST_DATA_DIR}/drone_survey_${test_name}_test"
    
    echo_info "Recording ${test_name} test data for ${duration} seconds..."
    
    # REMOVE existing directory if it exists
    if [ -d "${bag_name}" ]; then
        echo_warning "Removing existing data: ${bag_name}"
        rm -rf "${bag_name}"
    fi
    
    # Topics to record, only topics relevant to TASK 1
    local topics=(
        "/drone/gt_odom"
        "/drone/sonar" 
        "/drone/laserscan"
        "/mission/goals"
        "/grid_map"
        "/mission/path"
        "/visualization_marker"
    )
    
    # Start recording FIRST
    ros2 bag record -o "${bag_name}" "${topics[@]}" &
    local record_pid=$!
    
    sleep 3  # Give recording time to start
    
    # PUBLISH GOALS AFTER RECORDING STARTS
    publish_test_goals "${test_name}"
    
    sleep 2  # Let goals publish properly
    
    # Start mission
    echo_info "Starting mission via service call..."
    ros2 service call /mission/control std_srvs/srv/SetBool "{data: true}" &
    
    # Record for specified duration
    echo_info "Recording for ${duration} seconds..."
    sleep "${duration}"
    
    # PUBLISH GOALS AGAIN (in case first one was missed)
    publish_test_goals "${test_name}"
    
    sleep 2
    
    # Stop mission
    echo_info "Stopping mission..."
    ros2 service call /mission/control std_srvs/srv/SetBool "{data: false}" &
    
    sleep 2
    
    # Stop recording with improved error handling
    if ps -p ${record_pid} > /dev/null 2>&1; then
        echo_info "Stopping ROSbag recording (PID: ${record_pid})"
        kill -INT ${record_pid}
        wait ${record_pid} 2>/dev/null || true
    else
        echo_warning "Recording process ${record_pid} already stopped"
    fi
    
    if [ -d "${bag_name}" ]; then
        echo_success "Test data recorded: ${bag_name}"
        ros2 bag info "${bag_name}"
        return 0
    else
        echo_error "Failed to record test data"
        return 1
    fi
}

# Function to check if system is ready
check_system() {
    echo_info "Checking ROS system..."
    
    if ! ros2 node list | grep -q "drone_controller"; then
        echo_error "drone_controller node not found!"
        echo_info "Please run: ros2 run a3_skeleton drone_node"
        return 1
    fi
    
    # Check required topics
    local required_topics=("/drone/gt_odom" "/drone/sonar")
    for topic in "${required_topics[@]}"; do
        if ! ros2 topic list | grep -q "${topic}"; then
            echo_warning "Topic ${topic} not found"
        fi
    done
    
    echo_success "System appears ready"
    return 0
}

# Function to collect all test data
collect_all_test_data() {
    echo_info "=== Collecting Unit Test Data for a3_skeleton ==="
    
    if ! check_system; then
        echo_error "System check failed"
        exit 1
    fi
    
    # Test 1: GridMap Creation (60 seconds)
    echo_info "=== Test 1: GridMap Creation ==="
    record_test_data "gridmap" 60
    sleep 3
    
    # Test 2: Traversability Analysis (60 seconds)
    echo_info "=== Test 2: Goal Traversability ==="
    record_test_data "traversability" 60
    sleep 3
    
    # Test 3: TSP Solver (40 seconds)
    echo_info "=== Test 3: TSP Optimisation ==="
    record_test_data "tsp" 40
    
    echo_success "All test data collection completed!"
    echo_info "Data location: ${TEST_DATA_DIR}"
    ls -la "${TEST_DATA_DIR}"/
}

# Run unit tests (DOES NOT WORK)
run_unit_tests() {
    echo_info "Building and running unit tests (ROS2 standard way)..."
    
    # Navigate to workspace root
    cd "${SCRIPT_DIR}/../.."
    
    # Clean build with testing enabled
    echo_info "Clean building package with tests..."
    rm -rf build/a3_skeleton install/a3_skeleton log/latest*
    
    colcon build --packages-select a3_skeleton --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -ne 0 ]; then
        echo_error "Build failed!"
        return 1
    fi
    
    # Source workspace
    source install/setup.bash
    
    # Run tests using proper ROS2 method
    echo_info "Running tests with colcon test..."
    colcon test --packages-select a3_skeleton --event-handlers console_direct+
    
    # Show detailed results
    echo_info "=== Test Results ==="
    colcon test-result --all --verbose
    
    # Show what test executables were created
    echo_info "Available test executables:"
    find ./build/a3_skeleton -name "*test*" -executable -type f 2>/dev/null || true
    
    # Parse test logs if available
    if [ -f "log/latest_test/a3_skeleton/stdout.log" ]; then
        echo_info "=== Test Output Summary ==="
        echo "Last 30 lines of test output:"
        tail -30 log/latest_test/a3_skeleton/stdout.log
    fi
    
    echo_success "Unit tests completed!"
}

# Validate test data
validate_test_data() {
    echo_info "Validating test data..."
    
    local bags=("gridmap" "traversability" "tsp")
    local all_valid=true
    
    for bag in "${bags[@]}"; do
        local bag_path="${TEST_DATA_DIR}/drone_survey_${bag}_test"
        if [ -d "${bag_path}" ]; then
            echo_success "✓ ${bag} test data found"
            
            # Check bag contents
            local info=$(ros2 bag info "${bag_path}" 2>/dev/null)
            if echo "${info}" | grep -q "/drone/gt_odom"; then
                echo_success "  ✓ Odometry data present"
            else
                echo_error "  ✗ Missing odometry data"
                all_valid=false
            fi
            
            if echo "${info}" | grep -q "/drone/sonar"; then
                echo_success "  ✓ Sonar data present"
            else
                echo_error "  ✗ Missing sonar data"
                all_valid=false
            fi
        else
            echo_error "✗ ${bag} test data missing"
            all_valid=false
        fi
    done
    
    if [ "$all_valid" = true ]; then
        echo_success "All test data validation passed!"
    else
        echo_error "Test data validation failed!"
        return 1
    fi
}

# Clean test data
cleanup_test_data() {
    echo_warning "This will delete all test data in ${TEST_DATA_DIR}"
    read -p "Continue? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "${TEST_DATA_DIR}"/*
        echo_success "Test data cleaned"
    fi
}

# Main function
main() {
    case "${1:-}" in
        "collect")
            collect_all_test_data
            ;;
        "validate")
            validate_test_data
            ;;
        "test")
            run_unit_tests
            ;;
        "clean")
            cleanup_test_data
            ;;
        *)
            echo "Usage: $0 {collect|validate|test|clean}"
            echo ""
            echo "Commands:"
            echo "  collect   - Collect all test data using ROSbag recording"
            echo "  validate  - Verify collected test data is complete"
            echo "  test      - Build and run unit tests"
            echo "  clean     - Remove all test data"
            echo ""
            echo "Workflow:"
            echo "  1. Start your drone simulation"
            echo "  2. ros2 run a3_skeleton drone_node"
            echo "  3. $0 collect"
            echo "  4. $0 test"
            ;;
    esac
}

main "$@"