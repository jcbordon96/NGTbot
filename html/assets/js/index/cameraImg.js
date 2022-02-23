const uri = "ws://192.168.4.1:9000";
const cameraSocket = new WebSocket(uri);

cameraSocket.addEventListener('message', (e) => {
    let ctx = canvas[0].getContext("2d");
    var height = body.height();
    var width = (height * 640) / 480;
    var menuWidth = body.width() - width-20;
    $('.menu').css('width', menuWidth+'px')
    
    let image = new Image(width,height);
    image.src = URL.createObjectURL(e.data);
    image.addEventListener("load", (e) => {
        ctx.drawImage(image, 0, 0, width, height);
    });
    
});
