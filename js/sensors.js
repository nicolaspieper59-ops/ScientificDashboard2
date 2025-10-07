// Capteurs : accélération Z, baromètre, lumière, son, magnétomètre
const Sensors = (function(){
  let accelZ = 0;
  function startAccelerometer(){
    if(window.DeviceMotionEvent){
      window.addEventListener('devicemotion', e=>{
        accelZ = e.accelerationIncludingGravity?.z || 0;
      });
    }
  }
  function getAccelZ(){ return accelZ; }

  // Placeholder pour autres capteurs
  let light=0, sound=0, magnet=0, pressure=0;
  function startOtherSensors(){
    // TODO : baromètre, luminosité, son, magnétomètre
  }

  return {startAccelerometer,getAccelZ,startOtherSensors};
})();
