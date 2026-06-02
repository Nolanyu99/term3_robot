const char* state_name()
{
  switch (follow_state) {
    case FollowState::Idle:
      return "idle";
    case FollowState::FollowLine:
      return "follow";
    case FollowState::CrossIntersection:
      return "intersection";
    case FollowState::LostLine:
      return "lost";
    default:
      return "unknown";
  }
}

void print_array(const char* label, const uint16_t* values)
{
  Serial.print(label);
  Serial.print("=[");

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i > 0) {
      Serial.print(',');
    }

    Serial.print(values[i]);
  }

  Serial.print(']');
}

void print_line_status()
{
  const bool found = line_detected;
  const int32_t position = found ? estimate_line_position() : -1;
  const int32_t error = position >= 0 ? position - LINE_CENTER : last_error;

  Serial.print("running=");
  Serial.print(running ? 1 : 0);

  Serial.print(" state=");
  Serial.print(state_name());

  Serial.print(" motoron=");
  Serial.print(motoron_ready ? 1 : 0);

  Serial.print(" found=");
  Serial.print(found ? 1 : 0);

  Serial.print(" junction=");
  if (found) {
    Serial.print(junctionTypeName(detect_junction_type()));
  } else {
    Serial.print("lost");
  }

  Serial.print(" line=");
  Serial.print(position);

  Serial.print(" error=");
  Serial.print(error);

  Serial.print(" last_side=");
  Serial.print(last_line_side);

  Serial.print(" motor=");
  Serial.print(last_left_command);
  Serial.print(',');
  Serial.print(last_right_command);

  Serial.print(' ');
  print_array("cal", calibrated_values);

  printIMUStatus();

  printEncoders();

  Serial.println();
}

void print_calibration()
{
  Serial.println("Calibration complete.");

  print_array("min", min_values);
  Serial.print(' ');
  print_array("max", max_values);
  Serial.println();

  Serial.println("Line following ready.");
}