import numpy as np
import time
import sys

# İstenilen hıza biz nesne tespit ve sürdürülebilir alan kısmından gelen kararlar ile belirleyeceğiz.
# Anlik hız bilgisi ise ROS üzerinden sağlanacak.


# Eksikler:
#  - Formülasyondaki örnekleme zamanı eklenmedi.
#  - K parametreleri güncellenmesi gerekli.


class pid:
    def __init__(self):
        self.speed_hatalar = np.zeros((7))
        self.steer_hatalar = np.zeros((3))
        self.i = 0
        self.pid_result = 0
        self.steer_output = 0
        self.timer = time.time()
        self.steer_Kp = 2    # ARACIN DİNAMİKLERİNE GÖRE DEĞİŞECEK KATSAYILAR
        self.steer_Ki = 3
        self.steer_Kd = 0.3
        self.speed_Kp = 1
        self.speed_Ki = 1
        self.speed_Kd = 1


    def speed_control(self, current_speed, wanted_speed=3):
        self.timer = time.time() - self.timer  # PID KONTROLDEKİ ÖRNEKLEME SÜRESİNİN HESABI

        hata = (
            wanted_speed - current_speed
        )  # İSTEDİĞİMİZ HIZ'DAN ARAÇTAN ANLIK OLARAK ALDIĞIMIZ HIZ ÇIKARTILARAK HATA HESABI
        if self.i == 7:
            self.i = 0
            print(
                "ISTENEN HIZ {} ANLIK HIZ {:6.2f} HATA {:6.2f} PID RESULT {:6.2f} Kp:{:.1f} Ki:{:.1f} Kd:{:.1f}".format(
                    wanted_speed, current_speed, hata, self.speed_output,self.speed_Kp,self.speed_Ki,self.speed_Kd
                )
            )
            sys.stdout.write("\033[F")
            sys.stdout.write("\033[K")

        self.speed_hatalar[self.i] = hata  # speed_Hatalarımızı bir dizide bufferlıyoruz.
        self.hata_toplami = np.sum(self.speed_hatalar)  # PID'de ki I icin hata toplami verisi.
        self.speed_output = (
            self.speed_Kp * (self.speed_hatalar[self.i])
            + self.speed_Ki * self.hata_toplami
            + self.speed_Kd * (self.speed_hatalar[self.i - 1] - self.speed_hatalar[self.i - 2])
        )  # Nihai PID çıkışımız.
        self.speed_output = self.speed_output / 100
        self.i += 1
        self.timer = time.time()
        return self.speed_output

    def steer_control(self, current_steer, wanted_steer=2):
        self.timer = time.time() - self.timer  # PID KONTROLDEKİ ÖRNEKLEME SÜRESİNİN HESABI

        hata = (
            wanted_steer - current_steer
        )  # İSTEDİĞİMİZ HIZ'DAN ARAÇTAN ANLIK OLARAK ALDIĞIMIZ HIZ ÇIKARTILARAK HATA HESABI
        if self.i == 3:
            self.i = 0
        
        print(
            "ISTENEN HIZ {} ANLIK HIZ {:6.2f} HATA {:6.2f} PID RESULT {:6.2f} Kp:{:.1f} Ki:{:.1f} Kd:{:.1f}".format(
                    wanted_steer, current_steer, hata, self.steer_output,self.steer_Kp,self.steer_Ki,self.steer_Kd
                )
        )
        sys.stdout.write("\033[F")
        sys.stdout.write("\033[K")

        self.steer_hatalar[self.i] = hata  # steer_Hatalarımızı bir dizide bufferlıyoruz.
        self.hata_toplami = np.sum(self.steer_hatalar)  # PID'de ki I icin hata toplami verisi.
        self.steer_output = (
            self.steer_Kp * (self.steer_hatalar[self.i])
            + self.steer_Ki * self.hata_toplami
            + self.steer_Kd * (self.steer_hatalar[self.i - 1] - self.steer_hatalar[self.i - 2])
        )  # Nihai PID çıkışımız.
        self.steer_output = self.steer_output / 5
        self.i += 1
        self.timer = time.time()
        return self.steer_output