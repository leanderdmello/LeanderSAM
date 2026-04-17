imu_packet_working.json -> baseline validated case
imu_result_working.json -> expected output for baseline case

imu_packet_transition_test.json -> validated case with modified transition matrix F
imu_result_transition_test.json -> expected output for modified transition case

Run commands:
julia imu_infer.jl examples/validated/imu_packet_working.json /tmp/out.json
julia imu_infer.jl examples/validated/imu_packet_transition_test.json /tmp/out.json
