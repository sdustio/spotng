# Style Guide

## 命名

- {{RetType}} const & {{ClassName}}::Get{{Member}}() const;
- bool {{ClassName}}::Build{{Object}}({{ObjectType &}}) \[const\];
- bool Func({{RetType &}}, arg1...)


## 指针

- 尽量使用引用
- 需要多态或者保存引用使用指针
- 用智能指针
- 不可以delete智能指针
