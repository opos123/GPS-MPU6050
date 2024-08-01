#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// Meng-include header yang diperlukan untuk kelas ini
#include <QObject>
#include <QSerialPort>
#include <QJsonDocument>
#include <QJsonObject>

// Deklarasi kelas MainWindow yang merupakan turunan dari QObject
class MainWindow : public QObject
{
    Q_OBJECT // Makro untuk mendukung mekanisme signal dan slot Qt

    // Mendeklarasikan properti dengan tipe double yang dapat diakses dari QML atau C++
    Q_PROPERTY(double yaw READ yaw NOTIFY yawChanged)
    Q_PROPERTY(double pitch READ pitch NOTIFY pitchChanged)
    Q_PROPERTY(double roll READ roll NOTIFY rollChanged)
    Q_PROPERTY(double accuracy READ accuracy NOTIFY accuracyChanged)
    Q_PROPERTY(double latitude READ latitude NOTIFY latitudeChanged)
    Q_PROPERTY(double longitude READ longitude NOTIFY longitudeChanged)

public:
    // Konstruktor eksplisit yang menerima objek parent, default-nya adalah nullptr
    explicit MainWindow(QObject *parent = nullptr);

    // Metode getter untuk properti yaw, pitch, roll, latitude, dan longitude
    double yaw() const { return m_yaw; }
    double pitch() const { return m_pitch; }
    double roll() const { return m_roll; }
    double accuracy() const { return m_accuracy; }
    double latitude() const { return m_latitude; }
    double longitude() const { return m_longitude; }

signals:
    // Signal yang akan diemit ketika nilai yaw berubah
    void yawChanged();
    // Signal yang akan diemit ketika nilai pitch berubah
    void pitchChanged();
    // Signal yang akan diemit ketika nilai roll berubah
    void rollChanged();
    // Signal yang akan diemit ketika nilai latitude berubah
    void accuracyChanged();
    void latitudeChanged();
    // Signal yang akan diemit ketika nilai longitude berubah
    void longitudeChanged();

private slots:
    // Slot yang akan dipanggil untuk membaca data dari port serial
    void readData();

private:
    QSerialPort m_serial; // Objek untuk mengelola komunikasi serial
    double m_yaw; // Variabel untuk menyimpan nilai yaw
    double m_pitch; // Variabel untuk menyimpan nilai pitch
    double m_roll; // Variabel untuk menyimpan nilai roll
    double m_accuracy;
    double m_latitude; // Variabel untuk menyimpan nilai latitude
    double m_longitude; // Variabel untuk menyimpan nilai longitude
};

#endif // MAINWINDOW_H
