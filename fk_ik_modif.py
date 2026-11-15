import numpy as np
import matplotlib.pyplot as plt


#matriks transformasi 4x4

def rotasi_z(theta_deg):
    """Rotasi terhadap sumbu Z (servo Coxa, putar kiri-kanan)."""
    t = np.radians(theta_deg)
    c = np.cos(t)
    s = np.sin(t)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])


def rotasi_pitch(theta_deg):
    #rotasi naik turun
    t = np.radians(theta_deg)
    c = np.cos(t)
    s = np.sin(t)
    return np.array([
        [ c, 0, -s, 0],
        [ 0, 1,  0, 0],
        [ s, 0,  c, 0],
        [ 0, 0,  0, 1]
    ])


def geser_x(panjang):
    #geser sepanjang sumbu X (panjang link)
    return np.array([
        [1, 0, 0, panjang],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


# FK

def hitung_fk(coxa, femur, tibia, th0, th1, th2):
    #mengembalikan posisi x y z
    T_coxa = rotasi_z(th0) @ geser_x(coxa)
    T_femur = rotasi_pitch(th1) @ geser_x(femur)
    T_tibia = rotasi_pitch(th2) @ geser_x(tibia)

    
    T1 = T_coxa
    T2 = T1 @ T_femur
    T3 = T2 @ T_tibia

    titik = [
        np.array([0, 0, 0]),
        T1[:3, 3],
        T2[:3, 3],
        T3[:3, 3]
    ]
    return titik


# IK

def hitung_ik(coxa, femur, tibia, x, y, z, siku_atas=True):
    #hitung sudut dari target x y z
    if femur == 0 or tibia == 0:
        print("IK tidak bisa dihitung kalau panjang Femur atau Tibia = 0.")
        return None

    th0 = np.arctan2(y, x)

    # jarak mendatar dari ujung coxa ke target
    r = np.sqrt(x ** 2 + y ** 2) - coxa

    # hukum cosinus
    D = (r ** 2 + z ** 2 - femur ** 2 - tibia ** 2) / (2 * femur * tibia)

    if D > 1 or D < -1:
        print("target di luar jangkauan")
        return None

    if siku_atas:
        th2 = np.arctan2(-np.sqrt(1 - D ** 2), D)
    else:
        th2 = np.arctan2(np.sqrt(1 - D ** 2), D)

    th1 = np.arctan2(z, r) - np.arctan2(tibia * np.sin(th2), femur + tibia * np.cos(th2))

    return np.degrees(th0), np.degrees(th1), np.degrees(th2)


# gui

def gambar_robot(titik, nama_file="hasil_visualisasi.png"):
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
    ax.set_title("Postur Kaki Robot")
    ax.legend()

    # supaya skala sumbu sama
    batas = max(max(abs(v) for v in xs + ys + zs), 1)
    ax.set_xlim(-batas, batas)
    ax.set_ylim(-batas, batas)
    ax.set_zlim(-batas, batas)



def tampilkan_hasil(titik):
    ujung = titik[-1]
    print("Posisi end-effector:")
    print(f"x = {ujung[0]:.2f} cm")
    print(f"y = {ujung[1]:.2f} cm")
    print(f"z = {ujung[2]:.2f} cm")


# main

def main():

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
        th0 = float(input("Sudut Coxa / yaw (derajat) [0 = bidang 2D]: ") or 0)
        th1 = float(input("Sudut Servo 1 - Femur (derajat): "))
        th2 = float(input("Sudut Servo 2 - Tibia (derajat): "))

        titik = hitung_fk(coxa, femur, tibia, th0, th1, th2)
        tampilkan_hasil(titik)
        gambar_robot(titik)

    elif pilihan == "2":
        x = float(input("Target X (cm): "))
        y = float(input("Target Y (cm): "))
        z = float(input("Target Z (cm): "))

        hasil = hitung_ik(coxa, femur, tibia, x, y, z)
        if hasil is None:
            return

        th0, th1, th2 = hasil
        print(f"Sudut Coxa   = {th0:.2f} derajat")
        print(f"Sudut Femur  = {th1:.2f} derajat")
        print(f"Sudut Tibia  = {th2:.2f} derajat")

        # cek balik pakai FK
        titik = hitung_fk(coxa, femur, tibia, th0, th1, th2)
        tampilkan_hasil(titik)
        gambar_robot(titik)

    else:
        print("Pilihan tidak valid.")


main()
