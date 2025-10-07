// Filtre de Kalman 2-états (position x, vitesse v)
const Kalman2 = (function(){
  let X = [0,0];   // [position, vitesse]
  let P = [[1000,0],[0,1000]]; 
  const Q = [[0.01,0],[0,0.01]]; // bruit du modèle

  function predict(dt, acc){
    X[0] += X[1]*dt;      // position = position + vitesse*dt
    X[1] += acc*dt;       // vitesse = vitesse + accélération*dt
    P[0][0] += dt*Q[0][0]; 
    P[1][1] += dt*Q[1][1];
  }

  function update(z, R){
    const H = [1,0]; 
    const y = z - (H[0]*X[0]+H[1]*X[1]); // innovation
    const S = P[0][0] + R; // innovation covariance
    const K = [P[0][0]/S, P[1][0]/S]; // gain
    X[0] += K[0]*y;
    X[1] += K[1]*y;
    P[0][0] = (1-K[0])*P[0][0];
    P[0][1] = (1-K[0])*P[0][1];
    P[1][0] = (1-K[1])*P[1][0];
    P[1][1] = (1-K[1])*P[1][1];
    return K[0]; // retourne gain
  }

  function getState(){ return {position:X[0], vitesse:X[1], P:P[0][0]}; }
  function reset(){ X=[0,0]; P=[[1000,0],[0,1000]]; }

  return {predict, update, getState, reset};
})();
               
