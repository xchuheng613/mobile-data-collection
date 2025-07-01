from multiprocessing.managers import BaseManager

class FooManager(BaseManager): pass
FooManager.register('Foo')

if __name__ == '__main__':
    manager = FooManager(address=('localhost', 10000), authkey=b'abc')
    manager.connect()
    f = manager.Foo()
    f.ping()
