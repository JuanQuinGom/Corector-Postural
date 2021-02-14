#ifndef RATE_HPP
#define RATE_HPP

class Rate{
private:
  uint32_t pivot;
  const double hz;
public:

  Rate(double _hz): hz(_hz){
    
  }

  void sleep(){
    while((millis() - (1000/hz))<pivot);
    pivot = millis();
  }
    float time_window() const{
    return (1.0/hz);
  }
   uint32_t get_rate(){
    return hz;
  }
  bool has_passed(){
    if ((millis() - (1000/hz))>=pivot){
      pivot = millis();
      return true;
    }
    return false;
  }
};

#endif //RATE_HPP
