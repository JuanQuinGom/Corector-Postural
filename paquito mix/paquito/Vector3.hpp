#ifndef VECTOR3_HPP
#define VECTOR3_HPP

template<class T>
class Vector3{
private:
  using thisClass = Vector3<T>;
public:
  using datatype = T;
  T data[3];
  T &x = data[0];
  T &y = data[1];
  T &z = data[2];
  Vector3()  : data{0, 0, 0} {};
  Vector3(T _x, T _y, T _z) : data{_x, _y, _z} {};
  bool operator == (const thisClass& o) const{
    return
      x == o.x &&
      y == o.y &&
      z == o.z;
  }
  thisClass& operator= (const thisClass& o) {
    x = o.x;
    y = o.y;
    z = o.z;
  };
  Vector3(const thisClass& o) {
    x = o.x;
    y = o.y;
    z = o.z;
  };
  Vector3(const thisClass&& o){
    x = o.x;
    y = o.y;
    z = o.z;
  };
  String to_string() const {
    String r = "{";
    r += x;
    r += ", ";
    r += y;
    r += ", ";
    r += z;
    r+="}";
    return r;
  }
  Vector3<T> &operator *= (const T& escalar){
    for(auto& d : data){
      d *= escalar;
    }
    return *this;
  }
};

#endif //VECTOR3_HPP
