# Style Guide

## 命名

- {{RetType}} const & {{ClassName}}::Get{{Member}}() const;  // Get instance member
- static bool {{ClassName}}::Make{{Object}}({{ObjectType &}}) \[const\]; // Make an instance
- bool Func({{RetType &}}, arg1...);
  - Trans //inplace
  - Calc //assign result to ref
  - ...


## 指针

- 尽量使用引用
- 需要多态或者保存引用使用指针
- 用智能指针
- 不可以delete智能指针
