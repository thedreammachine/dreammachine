base_interval = null

var songDictionary = {};
songDictionary["cirrus"] = "Bonobo - Cirrus.wav";
songDictionary["recurring"] = "Bonobo - Recurring.wav";
songDictionary["silver"] = "Bonobo - Silver.wav";
songDictionary["never"] = "Rick Astley - Never Gonna Give You Up.wav";
songDictionary["baby"] = "Sir MixALot - Baby Got Back.wav";
songDictionary["beautiful"] = "Uppermost - Beautiful Light.wav";
songDictionary["energy"] = "Uppermost - Energy.wav";

$('#myTab a').click(function (e) {
  e.preventDefault()
  // alert($(this).attr('href'));
  if($(this).attr('href') == "#voice-tab") {
    var params = {type:"flip_voice_handler", turn:"on"};
    $.get("/", params); 
  } else {
    var params = {type:"flip_voice_handler", turn:"off"};
    $.get("/", params);  
  }

  $(this).tab('show')
})

function base_command_start(x, y, z) {
    //alert("hello");
    //alert("x:" + x + "y:" + y + "z:" + z);
    var params = {type:"base", x:x, y:y, z:z};
    // alert(params.type + params.x);
    base_interval = setInterval(function() {
        $.get("/", params); 
        }, 50
    );
}

function base_command_end() {
	clearInterval(base_interval);
}

$(document).keypress(function(e){
    if (e.keyCode == 38) { 
        var params = {type:"base", x:0.1, y:0, z:0};
        base_interval = setInterval(function() {
            $.get("/", params); 
            }, 500
        );
        return false;
    }
});

$(document).keypress(function(e){
    if (e.keyCode == 37) { 
        var params = {type:"base", x:0, y:0, z:0.3};
        base_interval = setInterval(function() {
            $.get("/", params); 
            }, 500
        );
        return false;
    }
});

$(document).keypress(function(e){
    if (e.keyCode == 39) { 
        var params = {type:"base", x:0, y:0, z:-0.3};
        base_interval = setInterval(function() {
            $.get("/", params); 
            }, 500
        );
        return false;
    }
});

$(document).keypress(function(e){
    if (e.keyCode == 40) { 
        var params = {type:"base", x:-0.1, y:0, z:0};
        base_interval = setInterval(function() {
            $.get("/", params); 
            }, 500
        );
        return false;
    }
});

$("#forward-command").mousedown(function() {
    base_command_start(0.1, 0, 0);
});

$("#forward-command").mouseup(function() {
    base_command_end();
});

$("#music-dropdown li").click(function() {
    var songAbbr = $(this).attr('id');
    var songFilename = songDictionary[songAbbr];
    var params = {type:"song", song:songFilename};
    $.get("/", params); 
});

$("#play-music").click(function() {
    var params = {type:"music", play:"play"};
    $.get("/", params); 
});

$("#stop-music").click(function() {
    var params = {type:"music", play:"stop"};
    $.get("/", params); 
});

var ros = new ROSLIB.Ros({
    url : 'ws://softshell.cs.washington.edu:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/voice_handler/voice_actions',
    messageType : 'std_msgs/String'   
});

listener.subscribe(function(message) {
    console.log('Received message on ' + listener.name + ': ' + message.data);
    $("#command-list").prepend('<li class="list-group-item">' + message.data + '</li>');
    // listener.unsubscribe();
});

console.log(listener);

// $("#stop-music").click(function() {
//     clearInterval(base_interval);
// });

// var ros = new ROSLIB.Ros({
//     url : 'localhost:31337'
// });

// ros.on('connection', function() {
//     console.log('Connected to websocket server.');
// });

// ros.on('error', function(error) {
//     console.log('Error connecting to websocket server: ', error);
// });

// ros.on('close', function() {
//     console.log('Connection to websocket server closed.');
// });

// // var listener = new ROSLIB.Topic({
// //     ros : ros,
// //     name : '/voice_handler/voice_actions',
// //     messageType : 'std_msgs/String'
// // });

// // listener.subscribe(function(message) {
// //     console.log('Received message on ' + listener.name + ': ' + message.data);
// //     alert("hello");
// //     listener.unsubscribe();
// // });

// ros.getParams(function(params) {
//     console.log(params);
// });