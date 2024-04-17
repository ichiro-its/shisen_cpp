#ifndef SHISEN_CPP__CONFIG__GRPC__CALL_DATA_BASE_HPP_
#define SHISEN_CPP__CONFIG__GRPC__CALL_DATA_BASE_HPP_

namespace shisen_cpp
{
class CallDataBase
{
public:
  CallDataBase();

  virtual void Proceed() = 0;

protected:
  virtual void WaitForRequest() = 0;
  virtual void HandleRequest() = 0;
};
}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CONFIG__GRPC__CALL_DATA_BASE_HPP_
