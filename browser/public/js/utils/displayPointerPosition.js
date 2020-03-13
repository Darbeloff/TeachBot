function writeMessage(canvas, message) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.font = '30pt Calibri';
    ctx.fillStyle = 'black';
    ctx.fillText(message, 10, 30);
}

function getMousePos(canvas, evt) {
    var rect = canvas.getBoundingClientRect();
    return {
      x: evt.clientX - rect.left,
      y: evt.clientY - rect.top
    };
}

// image.style.display = 'none';
canvas_obj.style.display = 'initial';
var ctx = canvas_obj.getContext('2d');
canvas_obj.addEventListener('mousemove', function(evt) {
    var mousePos = getMousePos(canvas_obj, evt);
    var message = 'Mouse position: ' + (100*mousePos.x/canvas_obj.width).toFixed(1) + '%, ' + (100*mousePos.y/canvas_obj.height).toFixed(1) +'%';
    writeMessage(canvas_obj, message);
}, false);