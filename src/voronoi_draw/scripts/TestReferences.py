class class1:
    var = 0

    def move(self):
        self.var += 1

class class2:
    class1Handle = 0

    def __init__(self, friend) -> None:
        self.class1Handle = friend

    def show(self):
        print(self.class1Handle.var)

ao1 = class1()
ao2 = class2(ao1)

while 1:
    ao1.move()
    ao2.show()