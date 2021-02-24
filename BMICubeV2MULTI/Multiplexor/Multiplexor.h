void PCA9546AD(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // PCA address is 0x70
  Wire.write(1 << bus);          // sEnd byte to select bus
  Wire.endTransmission();
}
