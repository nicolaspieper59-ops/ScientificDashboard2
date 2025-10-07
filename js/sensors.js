const Sensors = (function(){
  let accel = {x:0,y:0,z:0};
  let light=0, sound=0, magnet=0, pressure=0, ultraSon=0, infraRouge=0;

  function startAccelerometer(){
    if(window.DeviceMotionEvent){
      window.addEventListener('devicemotion', e=>{
        accel.x = e.accelerationIncludingGravity?.x || 0;
        accel.y = e.accelerationIncludingGravity?.y || 0;
        accel.z = e.accelerationIncludingGravity?.z || 0;
      });
    }
  }

  function startLightSensor(){
    if('AmbientLightSensor' in window){
      try{
        const sensor = new AmbientLightSensor();
        sensor.onreading = ()=> light = sensor.illuminance;
        sensor.start();
      } catch(e){ console.warn('Light sensor non dispo'); }
    }
  }

  function startMagnetometer(){
    if('Magnetometer' in window){
      try{
        const sensor = new Magnetometer();
        sensor.onreading = ()=> magnet = sensor.x; 
        sensor.start();
      } catch(e){ console.warn('Magnétomètre non dispo'); }
    }
  }

  function startBarometer(){
    if('PressureSensor' in window){
      try{
        const sensor = new PressureSensor();
        sensor.onreading = ()=> pressure = sensor.pressure;
        sensor.start();
      } catch(e){ console.warn('Baromètre non dispo'); }
    }
  }

  function startMicrophone(){
    navigator.mediaDevices.getUserMedia({audio:true}).then(stream=>{
      const audioCtx = new (window.AudioContext||window.webkitAudioContext)();
      const source = audioCtx.createMediaStreamSource(stream);
      const analyser = audioCtx.createAnalyser();
      source.connect(analyser);
      const data = new Uint8Array(analyser.frequencyBinCount);
      setInterval(()=>{ analyser.getByteTimeDomainData(data); sound = Math.max(...data); }, 200);
    }).catch(()=>{ console.warn('Microphone non dispo'); });
  }

  // Placeholder : infra-rouge et ultra-son via Bluetooth ou capteurs spéciaux
  function startInfraUltrasons(){ infraRouge=0; ultraSon=0; }

  function startAll(){
    startAccelerometer();
    startLightSensor();
    startMagnetometer();
    startBarometer();
    startMicrophone();
    startInfraUltrasons();
  }

  function getAll(){
    return {
      accel, light, sound, magnet, pressure, infraRouge, ultraSon
    };
  }

  return {startAll, getAll};
})();
    
