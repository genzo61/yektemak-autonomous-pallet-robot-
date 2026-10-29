"""Launch yardimcisi: zaten calisan bir Gazebo varsa baslatmayi durdurur.

Eski bir gzserver acik kalmissa yeni launch kendi Gazebo'sunu acamaz ve robotlari
ESKI dunyaya ekler. Robotlar baska bir dunyanin haritasiyla konum bulmaya calisir,
sacma yerlerde gorunur ve tikanir. Bunu sessizce yasamak yerine acikca durduruyoruz.
"""

import subprocess

from launch.actions import LogInfo, Shutdown


def check_no_running_gazebo(context, *args, **kwargs):
    try:
        out = subprocess.run(["pgrep", "-x", "gzserver"], capture_output=True, text=True).stdout.split()
    except FileNotFoundError:
        return []
    if not out:
        return []
    msg = ("\n\n*** ZATEN CALISAN BIR GAZEBO VAR (gzserver pid: " + ", ".join(out) + ") ***\n"
           "Robotlar o eski dunyaya eklenir ve harita ile dunya uyusmaz.\n"
           "Once kapatin:   killall -9 gzserver gzclient\n"
           "sonra launch'i tekrar calistirin.\n")
    return [LogInfo(msg=msg), Shutdown(reason="Calisan eski Gazebo bulundu")]
