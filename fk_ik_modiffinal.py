import numpy as np
import matplotlib.pyplot as plt


# matriks transformasi 4x4

def matriks_coxa(theta_deg, panjang):
    # rotasi putar kiri kanan (sumbu Z), terus geser sepanjang link
    theta = np.radians(theta_deg)
    c = np.cos(theta)
    s = np.sin(theta)

    matriks = np.array([
        [c, -s, 0, panjang * c],
        [s,  c, 0, panjang * s],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])
    return matriks


def matriks_transformasi(theta_deg, panjang):
    # rotasi naik turun buat femur dan tibia, terus geser sepanjang link
    # kalau coxa 0 hasilnya sama kayak versi 2D (y jadi z)
    theta = np.radians(theta_deg)
    c = np.cos(theta)
    s = np.sin(theta)

    matriks = np.array([
        [c, 0, -s, panjang * c],
        [0, 1,  0, 0],
        [s, 0,  c, panjang * s],
        [0, 0,  0, 1]
    ])
    return matriks


# FK

def hitung_fk(coxa, femur, tibia, theta0, theta1, theta2):
    # return 4 titik: pangkal, ujung coxa, ujung femur, ujung tibia
    T0 = matriks_coxa(theta0, coxa)
    T1 = matriks_transformasi(theta1, femur)
    T2 = matriks_transformasi(theta2, tibia)

    # urutan kali jangan dibalik
    T01 = T0 @ T1
    T = T0 @ T1 @ T2

    pangkal = (0.0, 0.0, 0.0)
    ujung_coxa = (T0[0, 3], T0[1, 3], T0[2, 3])
    ujung_femur = (T01[0, 3], T01[1, 3], T01[2, 3])
    ujung_tibia = (T[0, 3], T[1, 3], T[2, 3])

    return [pangkal, ujung_coxa, ujung_femur, ujung_tibia]


# IK

def hitung_ik(coxa, femur, tibia, x, y, z):
    # hitung sudut dari target x y z, kalau kejauhan return list kosong
    if femur == 0:
        print("IK tidak bisa dihitung kalau panjang Femur = 0.")
        return []

    # sudut coxa, arah target dilihat dari atas
    theta0 = np.degrees(np.arctan2(y, x))

    # jarak mendatar dari ujung coxa ke target
    r = np.sqrt(x ** 2 + y ** 2) - coxa
    jarak = np.sqrt(r ** 2 + z ** 2)

    # kalau tibia 0 cuma bisa nyampe jarak segitu aja (= panjang femur)
    if tibia == 0:
        if abs(jarak - femur) > 0.01:
            print("PERINGATAN: target di luar jangkauan (out of workspace)!")
            return []
        theta1 = np.degrees(np.arctan2(z, r))
        return [(theta0, theta1, 0.0)]

    # hukum cosinus
    D = (r ** 2 + z ** 2 - femur ** 2 - tibia ** 2) / (2 * femur * tibia)

    if D > 1 or D < -1:
        print("PERINGATAN: target di luar jangkauan (out of workspace)!")
        return []

    solusi = []
    # ada 2 solusi, siku atas sama siku bawah
    for tanda in [1, -1]:
        theta2 = np.arctan2(tanda * np.sqrt(1 - D ** 2), D)
        theta1 = np.arctan2(z, r) - np.arctan2(tibia * np.sin(theta2), femur + tibia * np.cos(theta2))
        solusi.append((theta0, np.degrees(theta1), np.degrees(theta2)))

    return solusi


# gambar

def gambar_robot(titik, nama_file="hasil_visualisasi_3d.png"):
    xs = [p[0] for p in titik]
    ys = [p[1] for p in titik]
    zs = [p[2] for p in titik]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(xs, ys, zs, "o-", linewidth=3)
    ax.scatter(xs[-1], ys[-1], zs[-1], color="red", s=80, label="End-effector")

    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_zlabel("Z (cm)")
    ax.set_title("Postur Kaki Robot 3D")
    ax.legend()

    # supaya skala sumbu sama
    batas = max(max(abs(v) for v in xs + ys + zs), 1)
    ax.set_xlim(-batas, batas)
    ax.set_ylim(-batas, batas)
    ax.set_zlim(-batas, batas)

    plt.show()


# main

def main():
    print("=== FK & IK Kaki Robot 3D (Coxa - Femur - Tibia) ===")

    # panjang link (cm), dari 2 digit terakhir NIU dan NIF
    femur = float(input("Panjang Femur (cm): "))
    tibia = float(input("Panjang Tibia (cm): "))
    coxa_str = input("Panjang Coxa (cm) [Enter = 0]: ")
    if coxa_str == "":
        coxa = 0.0
    else:
        coxa = float(coxa_str)

    print("\nMenu:")
    print("1. Forward Kinematics (FK)")
    print("2. Inverse Kinematics (IK)")
    pilihan = input("Pilih (1/2): ")

    if pilihan == "1":
        # sudut dalam derajat, CCW positif, CW negatif
        theta0_str = input("Sudut Coxa / yaw (derajat) [Enter = 0]: ")
        if theta0_str == "":
            theta0 = 0.0
        else:
            theta0 = float(theta0_str)
        theta1 = float(input("Sudut Servo 1 (derajat): "))
        theta2 = float(input("Sudut Servo 2 (derajat): "))

        titik = hitung_fk(coxa, femur, tibia, theta0, theta1, theta2)
        print("Posisi end-effector:")
        print(f"x = {titik[-1][0]:.2f} cm")
        print(f"y = {titik[-1][1]:.2f} cm")
        print(f"z = {titik[-1][2]:.2f} cm")
        gambar_robot(titik)

    elif pilihan == "2":
        x = float(input("Target X (cm): "))
        y = float(input("Target Y (cm): "))
        z = float(input("Target Z (cm): "))

        solusi = hitung_ik(coxa, femur, tibia, x, y, z)
        if len(solusi) == 0:
            return

        for i in range(len(solusi)):
            print(f"Solusi {i + 1}: Coxa = {solusi[i][0]:.2f}, Servo 1 = {solusi[i][1]:.2f}, Servo 2 = {solusi[i][2]:.2f} (derajat)")

        # cek balik pakai FK, solusi pertama yang digambar
        titik = hitung_fk(coxa, femur, tibia, solusi[0][0], solusi[0][1], solusi[0][2])
        print(f"Cek FK solusi 1: x = {titik[-1][0]:.2f}, y = {titik[-1][1]:.2f}, z = {titik[-1][2]:.2f}")
        gambar_robot(titik)

    else:
        print("Pilihan tidak valid.")


main()
