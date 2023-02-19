#!python3

import os
from pprint import pprint


def walk_dir(path):

    duplicates = {}

    for f in os.listdir(path):
        if os.path.isdir(f):
            walk_dir(os.path.join(path, f))
            continue

        # Отрезаем циферку от самого имени файла
        name, number = os.path.splitext(f)

        # Убежаемся, что то, что мы отрезали - реально циферка
        try:
            number = int(number.strip("."))
        except Exception as e:
            # Не циферка, идем дальше
            continue

        # Добавляем список для этого файла
        if name in duplicates.keys():
            duplicates[name].append(f)
        else:
            duplicates[name] = [f]

    # Ок! собрали все дубликаты
    # Теперь пойдем их фиксить
    for basename, variants in duplicates.items():
        print("Фиксю файл %s" % os.path.join(path, basename))

        # Сортируем дубликаты по циферке
        variants.sort(key=lambda item: int(os.path.splitext(item)[1].strip(".")))

        target = os.path.join(path, basename)
        source = os.path.join(path, variants[-1])

        # Если такой файл без циферки уже есть, считаем что файл с циферкой это более свежая версия
        # Поэтому удаляем его
        if os.path.isfile(target):
            os.remove(target)

        # Переименовываем наш файл как правильный
        os.rename(source, target)

        # Сносим все остальное (кроме последнего)
        for elem in variants[:-1]:
            os.remove(os.path.join(path, elem))



def main(argv):
    root_dir = argv[0]
    walk_dir(root_dir)


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        exit(main(sys.argv[1:]))
    else:
        exit(main(["."]))
