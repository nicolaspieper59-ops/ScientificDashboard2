let rituelActif = true;
const applicateurs = {
  physique: true,
  chimie: true,
  svt: true
};

function amplifierSon(grandeur, valeur) {
  if (!rituelActif) return;
  const ctx = new (window.AudioContext || window.webkitAudioContext)();
  const osc = ctx.createOscillator();
  const gain = ctx.createGain();

  let freq = 440;
  switch (grandeur) {
    case 'vitesse': freq = 100 + valeur * 5; break;
    case 'puissance': freq = 300 + valeur * 0.1; break;
    case 'lune': freq = 220 + valeur * 2; break;
    case 'cardiaque': freq = 60 + valeur; break;
  }

  osc.frequency.value = freq;
  osc.type = 'sine';
  gain.gain.value = 0.05;

  osc.connect(gain);
  gain.connect(ctx.destination);
  osc.start();
  osc.stop(ctx.currentTime + 0.5);
}

function amplifierVisuel(id, intensité = 1) {
  if (!rituelActif) return;
  const el = document.getElementById(id);
  if (!el) return;
  el.style.animation = `pulse ${2 / (intensité || 1)}s infinite`;
  el.classList.add('pulsation');
}

function amplifier(grandeurs) {
  if (applicateurs.physique) {
    amplifierSon('vitesse', grandeurs.vitesse);
    amplifierVisuel('vitesse', grandeurs.vitesse / 100);
  }
  if (applicateurs.chimie) {
    amplifierSon('puissance', grandeurs.puissance);
    amplifierVisuel('puissance', grandeurs.puissance / 500);
  }
  if (applicateurs.svt) {
    amplifierSon('cardiaque', grandeurs.cardiaque);
    amplifierVisuel('lune-phase', grandeurs.phase / 100);
  }
}

function testAmplification() {
  const vitesse = 42.5;
  const puissance = 180.0;
  const phase = 98.7;
  const cardiaque = 72;

  document.getElementById('vitesse').textContent = `Vitesse : ${vitesse} km/h`;
  document.getElementById('puissance').textContent = `Puissance : ${puissance} W`;
  document.getElementById('lune-phase').textContent = `Phase lunaire : ${phase}%`;
  document.getElementById('cardiaque').textContent = `Fréquence cardiaque : ${cardiaque} bpm`;

  amplifier({ vitesse, puissance, phase, cardiaque });
}

document.addEventListener('DOMContentLoaded', () => {
  document.getElementById('amplifier').onclick = testAmplification;

  document.getElementById('toggle-rituel').onclick = () => {
    rituelActif = !rituelActif;
    document.body.classList.toggle('rituel-off', !rituelActif);
    document.getElementById('toggle-rituel').textContent = rituelActif ? '🔮 Rituel : ON' : '🔋 Rituel : OFF';
  };

  ['physique','chimie','svt'].forEach(type => {
    document.getElementById(`toggle-${type}`).onclick = () => {
      applicateurs[type] = !applicateurs[type];
      document.getElementById(`toggle-${type}`).textContent =
        `${type === 'physique' ? '🧲' : type === 'chimie' ? '🧪' : '🌱'} ${type.charAt(0).toUpperCase() + type.slice(1)} : ${applicateurs[type] ? 'ON' : 'OFF'}`;
    };
  });
});
