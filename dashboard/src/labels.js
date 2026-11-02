export const ZONE_NAME = {
  inbound: "Mal kabul", storage: "Depo", production: "Üretim hattı",
  outbound: "Sevkiyat", charge: "Şarj",
};
export const ROBOT_COLOR = { robot1: "#2f6bff", robot2: "#f28c1b" };
export const robotColor = (n, i = 0) => ROBOT_COLOR[n] || ["#2f6bff", "#f28c1b", "#14a37f", "#9b4dca"][i % 4];
export const ROBOT_LABEL = (n) => n.replace(/^robot/, "Robot ");
export const STATE_TEXT = {
  idle: "Boşta", busy: "Görevde", charging: "Şarjda", estop: "Acil durdurma",
  error: "Hata", offline: "Bağlantı yok",
};
export const TASK_STATE = {
  queued: "Sırada", assigned: "Atandı", running: "Yürütülüyor", done: "Tamamlandı",
  failed: "Başarısız", cancelled: "İptal",
};
