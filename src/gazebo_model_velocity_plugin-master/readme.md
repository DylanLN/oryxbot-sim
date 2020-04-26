  <gazebo>
      <plugin name="base_drive_controller" filename="libgazebo_ros_model_velocity.so">
      <commandTopic>cmd_vel</commandTopic>
      <outputVelocityTopic>output_vel</outputVelocityTopic>
      <updateRate>50.0</updateRate>
      <commandTimeout>0.5</commandTimeout>
      <!-- Publish odometry as needed -->
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometryRate>20.0</odometryRate>
      <publishOdometryTf>true</publishOdometryTf>
      <robotBaseFrame>base_link</robotBaseFrame>
      <gaussianNoiseXY>0.02</gaussianNoiseXY>
      <gaussianNoiseYaw>0.02</gaussianNoiseYaw>
      <linearVelocityLimit>1.0</linearVelocityLimit>
      <angularVelocityLimit>3.0</angularVelocityLimit>
      <linearAccelerationLimit>1.0</linearAccelerationLimit>
      <angularAccelerationLimit>3.0</angularAccelerationLimit>
      <linearJerkLimit>5.0</linearJerkLimit>
      <angularJerkLimit>50.0</angularJerkLimit>
      </plugin>
  </gazebo>
