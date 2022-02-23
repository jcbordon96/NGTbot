
axios({
    method: 'get',
    url: 'http://192.168.4.1/state.json',
    })
    .then(function (response) {
        robotState.auto = response.data.auto;
        robotState.camera = response.data.camera;
        robotState.lights = response.data.flash;
        setInfo();
});

$('#openmenuConfBtn').click(()=>{
    menu.css('opacity', 0)
    menu.css('visibility', 'hidden')

    menuConf.css('opacity', 1)
    menuConf.css('visibility', 'visible')
})

$('#openmenuControlsBtn').click(()=>{
    if(!robotState.auto){
        menu.css('opacity', 0)
        menu.css('visibility', 'hidden')

        menuControls.css('opacity', 1)
        menuControls.css('visibility', 'visible')
    }else{
        Swal.fire({
            title: 'El modo autónomo esta activado para poder controlar el robot debe desactivarlo, desea desactivar el modo autonomo ahora?',
            showDenyButton: true,
            confirmButtonText: 'Desactivar',
            denyButtonText: `Cancelar`,
        }).then((result) => {
            /* Read more about isConfirmed, isDenied below */
            if (result.isConfirmed) {
                sendMessage({ action: 'auto', req: false })
                robotState.auto = false;
                menu.css('opacity', 0)
                menu.css('visibility', 'hidden')
                autoStatus.prop('checked', false)
                menuControls.css('opacity', 1)
                menuControls.css('visibility', 'visible')
            } else {

            }
        })
    }
    
})

$('#backBtn1').click(()=>{
    menuConf.css('opacity', 0)
    menuConf.css('visibility', 'hidden')

    menu.css('opacity', 1)
    menu.css('visibility', 'visible')
})

$('#backBtn2').click(()=>{
    menuControls.css('opacity', 0)
    menuControls.css('visibility', 'hidden')

    menu.css('opacity', 1)
    menu.css('visibility', 'visible')
})

socket = new WebSocket("ws://192.168.4.1:9001/");
socket.onmessage = function (event) {
    data = JSON.parse(event.data);
    if(data.type == "temp"){
        dataSplitted = data.data.split('/');
        console.log(dataSplitted)
        temp.text(dataSplitted[0]+'/'+dataSplitted[1]+'/'+dataSplitted[2])
        humidity.text(dataSplitted[3])
        ammonia.text(dataSplitted[4])
    }
};
setTimeout(() => {
    socket.send(JSON.stringify({ action: 'temp_req'}));
}, 500);
setInterval(() => {
    socket.send(JSON.stringify({ action: 'temp_req'}));
}, 60000);

function setInfo(){
    autoStatus.prop('checked', robotState.auto)
    lightStatus.prop('checked', robotState.lights)
    cameraStatus.prop('checked', robotState.camera)
}
function sendMessage(msg){
    socket.send(JSON.stringify(msg));
}

lightStatus.click((e)=>{
    if(lightStatus.prop('checked')){
        sendMessage({ action: 'flash', req: true })
    }else{
        sendMessage({ action: 'flash', req: false })
    }
})

autoStatus.click((e)=>{
    if(autoStatus.prop('checked')){
        sendMessage({ action: 'auto', req: true })
    }else{
        sendMessage({ action: 'auto', req: false })
    }
})

cameraStatus.click((e)=>{
    if(autoStatus.prop('checked')){
        sendMessage({ action: 'camera', req: true })
    }else{
        sendMessage({ action: 'camera', req: false })
    }
})

saveConfigBtn.click(()=>{
    sendMessage({ action: 'config', req: true })
    Swal.fire('Se guardó la configuración', '', 'success')
})

shutdownBtn.click(()=>{
    Swal.fire({
        title: 'Esta seguro que desea apagar el robot?',
        showDenyButton: true,
        confirmButtonText: 'Apagar',
        denyButtonText: `Cancelar`,
    }).then((result) => {
        if (result.isConfirmed) {
            sendMessage({ action: 'shutdown', req: true })
        } 
    })
    
})

rebootBtn.click(()=>{
    Swal.fire({
        title: 'Esta seguro que desea reiniciar el robot?',
        showDenyButton: true,
        confirmButtonText: 'Reiniciar',
        denyButtonText: `Cancelar`,
    }).then((result) => {
        if (result.isConfirmed) {
            sendMessage({ action: 'reboot', req: true })
        } 
    })
})

forwardBtn.bind('touchstart mousedown',(e)=>{
    e.preventDefault()
    sendMessage({ action: 'move', x: 1.0, z: 0 })
})

forwardBtn.bind('touchend touchcancel mouseup',()=>{
    sendMessage({ action: 'stop'})
})

backBtn.bind('touchstart mousedown',(e)=>{
    e.preventDefault()
    sendMessage({ action: 'move', x: -1.0, z: 0 })
})

backBtn.bind('touchend touchcancel mouseup',()=>{
    sendMessage({ action: 'stop'})
})

leftBtn.bind('touchstart mousedown',(e)=>{
    e.preventDefault()
    sendMessage({ action: 'move', x: 0, z: -1.0 })
})

leftBtn.bind('touchend touchcancel mouseup',()=>{
    sendMessage({ action: 'stop'})
})

rightBtn.bind('touchstart mousedown',(e)=>{
    e.preventDefault()
    sendMessage({ action: 'move', x: 0, z: 1.0 })
})

rightBtn.bind('touchend touchcancel mouseup',()=>{
    sendMessage({ action: 'stop'})
})
