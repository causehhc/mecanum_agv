def test(a, b):
    c = list(set(b).difference(set(a)))
    return c


def main():
    a = [[1, 1, 0], [2, 2, 0], [3, 3, 1]]
    b = [[1, 1, 1], [2, 2, 1], [3, 3, 1]]
    print(test(a[1], b[1]))


if __name__ == '__main__':
    main()
