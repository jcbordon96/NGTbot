
        axios({
            method: 'get',
            url: 'http://192.168.4.1/state.json',
            })
            .then(function (response) {
                autoRate.innerHTML = response.data.auto_rate;
                cameraRate.innerHTML = response.data.camera_rate;
                imuFlag.innerHTML = response.data.imu_flag
                autoState = response.data.auto;
                cameraState = response.data.camera;
                lightsState = response.data.flash;
                imuState = response.data.imu_req;
                setInfo();
            });
            websocket = new WebSocket("ws://192.168.4.1:9001/");
            websocket.onmessage = function (event) {
                data = JSON.parse(event.data);
    
                switch (data.type) { 
                    case 'temp':
                        temp.innerHTML = data.data
                    default:
                        console.error(
                            "unsupported event", data);
                }
            };
            //websocket.send(JSON.stringify({ action: 'temp_req'}));
            setInterval(() => {
                websocket.send(JSON.stringify({ action: 'temp_req'}));
            }, 60000);

            function sendMessage(msg){
                websocket.send(JSON.stringify(msg));
            }
    
            var autoState = false;
            var cameraState = false;
            var lightsState = false;
            var imuState = false;
            var rateAuto = 0;
            var rateCamera = 0;
            var flagImu = 0;
    
            function setInfo(){
                if(autoState){
                    autoSpan.innerHTML='Activado'
                    autoSpan.classList.remove('deactivated')
                    autoSpan.classList.add('activated')
                    autoOnBtn.classList.add('hide')
                    autoOffBtn.classList.remove('hide')
                }else{
                    autoSpan.innerHTML='Desactivado'
                    autoSpan.classList.add('deactivated')
                    autoSpan.classList.remove('activated')
                    autoOnBtn.classList.remove('hide')
                    autoOffBtn.classList.add('hide')
                }
                if(cameraState){
                    cameraSpan.innerHTML='Activado'
                    cameraSpan.classList.remove('deactivated')
                    cameraSpan.classList.add('activated')
                    cameraOnBtn.classList.add('hide')
                    cameraOffBtn.classList.remove('hide')
                }else{
                    cameraSpan.innerHTML='Desactivado'
                    cameraSpan.classList.add('deactivated')
                    cameraSpan.classList.remove('activated')
                    cameraOnBtn.classList.remove('hide')
                    cameraOffBtn.classList.add('hide')
                }
                if(lightsState){
                    lightsSpan.innerHTML='Activado'
                    lightsSpan.classList.remove('deactivated')
                    lightsSpan.classList.add('activated')
                    lightsOnBtn.classList.add('hide')
                    lightsOffBtn.classList.remove('hide')
                }else{
                    lightsSpan.innerHTML='Desactivado'
                    lightsSpan.classList.add('deactivated')
                    lightsSpan.classList.remove('activated')
                    lightsOnBtn.classList.remove('hide')
                    lightsOffBtn.classList.add('hide')
                }
                if(imuState){
                    imuSpan.innerHTML='Activado'
                    imuSpan.classList.remove('deactivated')
                    imuSpan.classList.add('activated')
                    imuOnBtn.classList.add('hide')
                    imuOffBtn.classList.remove('hide')
                }else{
                    imuSpan.innerHTML='Desactivado'
                    imuSpan.classList.add('deactivated')
                    imuSpan.classList.remove('activated')
                    imuOnBtn.classList.remove('hide')
                    imuOffBtn.classList.add('hide')
                }
    
            }
            var temp = document.getElementById('temp')
            var autoOnBtn = document.getElementById('autoOnBtn')
            var autoOffBtn = document.getElementById('autoOffBtn')
            var cameraOnBtn = document.getElementById('cameraOnBtn')
            var cameraOffBtn = document.getElementById('cameraOffBtn')
            var lightsOnBtn = document.getElementById('lightsOnBtn')
            var lightsOffBtn = document.getElementById('lightsOffBtn')
            var imuOnBtn = document.getElementById('imuOnBtn')
            var imuOffBtn = document.getElementById('imuOffBtn')
            var saveConfigBtn = document.getElementById('saveConfigBtn')
    
            var canvas = document.getElementById('msg')
            var showControls = document.getElementById('showControls')
            var hideControls = document.getElementById('hideControls')
            var moveControls = document.getElementById('moveControls')
            var buttonsConfigBox = document.getElementById('buttonsConfigBox')
            var buttonsConfigBox2 = document.getElementById('buttonsConfigBox2')
            var autoSpan = document.getElementById('autoSpan');
            var cameraSpan = document.getElementById('cameraSpan');
            var lightsSpan = document.getElementById('lightsSpan');
            var imuSpan = document.getElementById('imuSpan');
            var autoRate = document.getElementById('autoRate');
            var cameraRate = document.getElementById('cameraRate');
            var imuFlag = document.getElementById('imuFlag');
            var inputAuto = document.getElementById('inputAuto')
            var inputCamera = document.getElementById('inputCamera')
            var inputImu = document.getElementById('inputImu')
            var saveRateAuto = document.getElementById('saveRateAuto')
            var saveRateCamera = document.getElementById('saveRateCamera')
            var saveFlagImu = document.getElementById('saveFlagImu')
    
            saveRateAuto.addEventListener('click', () => {
                autoRate.innerHTML = inputAuto.value;
                websocket.send(JSON.stringify({ action: 'auto_rate', req: inputAuto.value }));
            })

            saveConfigBtn.addEventListener('click', ()=>{
                websocket.send(JSON.stringify({ action: 'config', req: true}))
            })
    
            saveRateCamera.addEventListener('click', () => {
                cameraRate.innerHTML = inputCamera.value;
                websocket.send(JSON.stringify({ action: 'camera_rate', req: inputCamera.value }));
            })
    
            saveFlagImu.addEventListener('click', () => {
                imuFlag.innerHTML = inputImu.value;
                websocket.send(JSON.stringify({ action: 'imu_flag', req: inputImu.value }));
            })
    
            showControls.addEventListener('click', ()=>{
                moveControls.style.display = 'block';
                showControls.style.display = 'none';
                hideControls.style.display = 'block';
                buttonsConfigBox.style.display = 'block';
                buttonsConfigBox2.style.display = 'block';
            })
    
            hideControls.addEventListener('click', ()=>{
                moveControls.style.display = 'none';
                showControls.style.display = 'block';
                hideControls.style.display = 'none';
                buttonsConfigBox.style.display = 'none';
                buttonsConfigBox2.style.display = 'none';
            })
        
            function opensocket(){
                let uri = "ws://" + "192.168.4.1" + ":9000";
                socket = new WebSocket(uri);
    
                socket.addEventListener('message', (e) => {
                    let ctx = canvas.getContext("2d");
                    let image = new Image();
                    image.src = URL.createObjectURL(e.data);
                    image.addEventListener("load", (e) => {
                        ctx.drawImage(image, 0, 0, canvas.width, canvas.height);
                    });
                    
                });
            }
    
            opensocket();
    
            autoOnBtn.addEventListener('click', ()=>{
                autoState = true;
                setInfo();
                websocket.send(JSON.stringify({ action: 'auto', req: true }));
            })
            
            autoOffBtn.addEventListener('click', ()=>{
                autoState = false
                setInfo();
                websocket.send(JSON.stringify({ action: 'auto', req: false }));
            })
    
            cameraOnBtn.addEventListener('click', ()=>{
                cameraState = true;
                setInfo();
                websocket.send(JSON.stringify({ action: 'camera', req: true }));
                
            })
            
            cameraOffBtn.addEventListener('click', ()=>{
                cameraState = false
                setInfo();
                websocket.send(JSON.stringify({ action: 'camera', req: false }));
                
            })
    
            lightsOnBtn.addEventListener('click', ()=>{
                lightsState = true;
                setInfo();
                websocket.send(JSON.stringify({ action: 'flash', req: true }));
                
            })
            
            lightsOffBtn.addEventListener('click', ()=>{
                lightsState = false
                setInfo();
                websocket.send(JSON.stringify({ action: 'flash', req: false }));
                
            })
    
            imuOnBtn.addEventListener('click', ()=>{
                imuState = true;
                setInfo();
                websocket.send(JSON.stringify({ action: 'imu_req', req: true }));
                
            })
            
            imuOffBtn.addEventListener('click', ()=>{
                imuState = false
                setInfo();
                websocket.send(JSON.stringify({ action: 'imu_req', req: false }));
                
            })
    
            var up = document.getElementById('up')
            var left = document.getElementById('left')
            var right = document.getElementById('right')
            var down = document.getElementById('down')
    
            left.ontouchstart = function (event) {
                event.preventDefault()
                websocket.send(JSON.stringify({ action: 'move', x: 0, z: -1.0 }));
            }
            right.ontouchstart = function (event) {
                event.preventDefault()
                websocket.send(JSON.stringify({ action: 'move', x: 0, z: 1.0 }));
            }
            up.ontouchstart = function (event) {
                event.preventDefault()
                websocket.send(JSON.stringify({ action: 'move', x: 1.0, z: 0 }));
            }
            down.ontouchstart = function (event) {
                event.preventDefault()
                websocket.send(JSON.stringify({ action: 'move', x: -1.0, z: 0 }));
            }
            left.ontouchend = function (event) {
                
                websocket.send(JSON.stringify({ action: 'stop' }));
            }
            right.ontouchend = function (event) {
                websocket.send(JSON.stringify({ action: 'stop' }));
            }
            up.ontouchend = function (event) {
                websocket.send(JSON.stringify({ action: 'stop' }));
            }
            down.ontouchend = function (event) {
                websocket.send(JSON.stringify({ action: 'stop' }));
            }
            //-----------
            left.onmousedown = function (event) {
                websocket.send(JSON.stringify({ action: 'move', x: 0, z: -1.0 }));
                console.log("left")
            }
            right.onmousedown = function (event) {
                websocket.send(JSON.stringify({ action: 'move', x: 0, z: 1.0 }));
            }
            up.onmousedown = function (event) {
                websocket.send(JSON.stringify({ action: 'move', x: 1.0, z: 0 }));
            }
            down.onmousedown = function (event) {
                websocket.send(JSON.stringify({ action: 'move', x: -1.0, z: 0 }));
            }
            left.onmouseup = function (event) {
                websocket.send(JSON.stringify({ action: 'stop' }));
            }
            right.onmouseup = function (event) {
                websocket.send(JSON.stringify({ action: 'stop' }));
            }
            up.onmouseup = function (event) {
                websocket.send(JSON.stringify({ action: 'stop' }));
            }
            down.onmouseup = function (event) {
                websocket.send(JSON.stringify({ action: 'stop' }));
            }