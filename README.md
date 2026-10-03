# ESP32 WEB-BASED INTELLIGENT CONTROL SYSTEM

Disusun oleh:
Daffa Armadhan Hidmi Ma’arif — 4.33.25.1.06
Kelas TI-2B / Semester Ganjil

Dosen Pengampu: Tahan Prahara, S.T., M.Kom.

PROGRAM STUDI TEKNOLOGI REKAYASA KOMPUTER
POLITEKNIK NEGERI SEMARANG
2026/2027

# Latar Belakang
Sistem kontrol umpan balik (feedback control) banyak dipakai pada peralatan elektronik untuk menjaga suatu besaran, seperti tegangan, suhu, atau kecepatan, agar tetap sesuai nilai yang diinginkan. Pada sistem seperti ini, nilai keluaran diukur kembali, dibandingkan dengan nilai acuan (set point), dan selisihnya (error) dipakai oleh kontroler untuk menentukan sinyal kendali.
ESP32 adalah mikrokontroler dengan Wi-Fi bawaan, ADC 12-bit, dan DAC 8-bit, sehingga dapat menjalankan seluruh rantai kontrol dalam satu perangkat: membaca sinyal, menghitung kontrol, menghasilkan sinyal keluaran, dan menyajikan antarmuka pemantauan melalui web server. Dengan MicroPython, implementasi menjadi lebih ringkas dan cocok untuk pembelajaran.
Praktikum ini membangun sistem kontrol tegangan berbasis ESP32 yang set point-nya dapat diubah melalui browser dan responsnya ditampilkan dalam grafik real-time. Plant yang dikendalikan adalah rangkaian low-pass RC. Kinerja kontroler dianalisis melalui error, overshoot, rise time, settling time, dan steady-state error. Saya memilih menggunakan controller PID karena sepertinya lebih sederhana dan juga rekomendasi dari AI. 
