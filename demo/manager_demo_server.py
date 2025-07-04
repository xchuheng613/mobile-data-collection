from multiprocessing.managers import BaseManager

class Foo:
    def ping(self):
        print("Ping received")

class FooManager(BaseManager): pass

FooManager.register('Foo', Foo)

if __name__ == '__main__':
    manager = FooManager(address=('localhost', 10000), authkey=b'abc')
    server = manager.get_server()
    print("Server started")
    server.serve_forever()
