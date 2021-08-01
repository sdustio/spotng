# Style Guide

## 命名

- {{RetType}} const & {{ClassName}}::Get{{Member}}() const;  // return an instance member
- static bool {{ClassName}}::Make{{Object}}({{ObjectType &}}) \[const\]; // Make an instance
- bool {{ComputeFunc}}Object({{RetType &}}, arg1...);
  - Trans //inplace
  - Calc //assign result to ref

## 指针

- 尽量使用引用
- 需要多态或者保存引用使用指针
- 用智能指针
- 不可以delete智能指针

## Eigen

- 类成员不使用 Eigen对象
- size(row * col) > 36，使用 dynamic matrix
- 声明(定义) dynamic matrix：
```cpp
// Prefer
MatrixXd m = Matrix<double, 18, 18>::Identity();
MatrixXd m = Matrix<double, 18, 18>::Zero();
```
