import dis

def test1():
    symbols = "@#$%^"
    codes = []
    for symbol in symbols:
        codes.append(ord(symbol))
    print(codes)


def test2():
    symbols = "@#$%^"
    codes = [ord(symbol) for symbol in symbols]
    print(codes)


if __name__ == '__main__':
    test1()
    test2()