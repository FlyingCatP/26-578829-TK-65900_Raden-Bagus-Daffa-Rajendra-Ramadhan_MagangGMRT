import numpy as np


def matriks_transformasi(theta_deg, panjang):
    """Membuat matriks homogen 3x3 untuk satu link (rotasi kemudian geser sepanjang link)."""
    theta = np.radians(theta_deg)  # derajat diubah dulu krena numpy pakai radian 
    c = np.cos(theta)
    s = np.sin(theta)

    matriks = np.array([
        [c, -s, panjang * c],
        [s,  c, panjang * s],
        [0,  0, 1]
    ])
    return matriks


def hitung_fk(femur, tibia, sudut1, sudut2):
    """Hitung posisi ujung Tibia (x, y) dengan Forward Kinematics."""
    T1 = matriks_transformasi(sudut1, femur)  # link femur
    T2 = matriks_transformasi(sudut2, tibia)  # link tibia

    # urutan perkalian tidak boleh dibalik: T1 dulu, baru T2
    T = T1 @ T2

    # posisi ujung ada di kolom terakhir matriks T
    x = T[0, 2]
    y = T[1, 2]
    return x, y


def main():
    # input panjang link (cm), dari 2 digit terakhir NIU dan NIF
    femur = float(input("Panjang femur dalam centimeter: "))
    tibia = float(input("Panjang tibia dalam centimeter: "))

    # input sudut servo (derajat), kalau berlawanan arah jarum jam positif,kalau searah jarum jam negatif
    sudut1 = float(input("Sudut Servo 1 : "))
    sudut2 = float(input("Sudut Servo 2 : "))

    x, y = hitung_fk(femur, tibia, sudut1, sudut2)

    print("Posisi end effector:")
    print(f"x = {x:.2f} cm")
    print(f"y = {y:.2f} cm")


main()