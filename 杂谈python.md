
# python知识
## 1. 类内使用局部函数
````
class MyClass:
    @classmethod
    def _generate_graph(cls, grid):
        print(f"Generating graph for grid: {grid}")

    def access_graph(self):
        # 类内调用直接访问
        self._generate_graph("Class Internal Call")
    
    @classmethod
    def access_graph_class(cls):
        cls._generate_graph("Class Method Internal Call")

# 示例
obj = MyClass()
obj.access_graph()            # 输出: Generating graph for grid: Class Internal Call
MyClass.access_graph_class()  # 输出: Generating graph for grid: Class Method Internal Call

````

## 2. def __init__(self):中初始化的变量与@classmethod可以访问的变量区别
### 2.1 __init__(self) 初始化方法
    __init__(self) 是 Python 中的构造函数，用于在创建类的实例时初始化实例的状态（即对象的属性）。
    它与类本身无关，专门用来初始化每个实例对象。
    
    作用：初始化实例化对象时的一些变量（实例属性）。
    访问的变量：__init__ 访问的是 实例变量，也就是通过 self 访问的变量。每个实例都可以有不同的属性。
````
class MyClass:
    def __init__(self, value):
        self.value = value  # 实例属性，只属于当前对象`
````
    在这个例子中，value 是每个实例的一个属性。每个 MyClass 的实例都可以有不同的 value。
### 2.2 @classmethod 类方法
    @classmethod 修饰符将方法转换为类方法，类方法的第一个参数是 cls，表示类本身。类方法可以访问 类变量，即属于类的属性，但不能直接访问实例变量，因为类方法不依赖于具体的实例。

- 作用：@classmethod 主要用于操作类本身的状态（类属性），而不是实例的状态。类方法只能访问类变量，不能直接访问实例变量。
- 访问的变量：类方法访问的是 类变量，也就是通过 cls 访问的变量，类的所有实例共享类变量。
````
class MyClass:
    class_variable = 42  # 类属性，所有实例共享

    @classmethod
    def show_class_variable(cls):
        print(f"Class variable: {cls.class_variable}")
````
- 在这个例子中，show_class_variable 是一个类方法，它访问的是类变量 class_variable。这个类变量对所有实例来说是共享的。
### 3.3 总结
1. 跟c++类似，如果需要根据不同对象区分的就在init下面；
2. 如果共享并且仅仅在对类本身起作用，就是用@classmthod
3. 如果想把一个函数放在一个跟函数本身没啥太大关系的类里，就用@staticmethod，
4. 上面如果在类内私有就使用__,并且无论是类级别变量还是实例化的变量，函数也同理