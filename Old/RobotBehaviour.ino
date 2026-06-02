void plant()
{
  closeBothGates();

  centreAfterRFID();

  stopMotors();
  delay(50);

  dispenseOne();

  delay(50);
}