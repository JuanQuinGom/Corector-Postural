#ifndef MEDIA_MOVIL_HPP
#define MEDIA_MOVIL_HPP

template<class T, int s>
class MediaMovil{
  T arr[s];
  int pos=0;
public:
  MediaMovil(T initial = T()){
    for(auto& a : arr){
      a=initial;
    }
  }
  T pass(T val){
    arr[pos] = val;
    pos = (pos+1)%s;
    double carry =0;
    for(auto& a : arr){
      carry+=a;
    }
    carry /= s;
    return carry;
  }
};

#endif //MEDIA_MOVIL_HPP
