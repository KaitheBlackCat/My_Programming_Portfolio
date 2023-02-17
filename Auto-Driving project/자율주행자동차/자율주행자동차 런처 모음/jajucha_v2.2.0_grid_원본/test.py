def checkingLineDetected(ListL, ListR, rows):
    for i in range(rows):
        if 0 < ListL[i] + ListR[i] < 60:
            e = 0
            print(str(i + 1) + "번째 선은 차선을 인식했습니다.")
        else:
            e = ListR[i] - ListL[i]
            print(str(i + 1)+ "번째 값이 정상입니다. 이를 출력합니다.")
            print("ListR[" + str(i) + "] = " + str(ListR[i]))
            print("ListL[" + str(i) + "] = " + str(ListL[i]))
            print("e = ListR[" + str(i) + "] - ListL["+ str(i) + "] = ")
            break

    return e


L = [2, 5, 18, 29, 39, 74, 68, 90, 192, 230, 319]
R = [1, 14, 15, 18, 20, 50, 70, 120, 214, 231, 319]

print(checkingLineDetected(L, R, 10))

