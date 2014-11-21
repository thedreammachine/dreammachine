$( document ).ready(function() {
    console.log("document ready!");

    base_interval = null

    var songDictionary = {};
    songDictionary["cirrus"] = "Bonobo - Cirrus.wav";
    songDictionary["recurring"] = "Bonobo - Recurring.wav";
    songDictionary["silver"] = "Bonobo - Silver.wav";
    songDictionary["never"] = "Rick Astley - Never Gonna Give You Up.wav";
    songDictionary["baby"] = "Sir MixALot - Baby Got Back.wav";
    songDictionary["beautiful"] = "Uppermost - Beautiful Light.wav";
    songDictionary["energy"] = "Uppermost - Energy.wav";

    // TAB
    $('#command-tabs a').click(function (e) {
      e.preventDefault()
      if($(this).attr('href') == "#voice-tab") {
        var params = {type:"flip_voice_handler", turn:"on"};
        $.get("/", params); 
      } else {
        var params = {type:"flip_voice_handler", turn:"off"};
        $.get("/", params);  
      }

      $(this).tab('show')
    })

    // Base movement
    $("#forward-command").mousedown(function() {
        base_command_start(0.1, 0, 0);
    });

    $("#forward-command").mouseup(function() {
        base_command_end();
    });

    $("#left-command").mousedown(function() {
        base_command_start(0, 0, 0.5);
    });

    $("#left-command").mouseup(function() {
        base_command_end();
    });

    $("#right-command").mousedown(function() {
        base_command_start(0, 0, -0.5);
    });

    $("#right-command").mouseup(function() {
        base_command_end();
    });

    $("#backward-command").mousedown(function() {
        base_command_start(-0.1, 0, 0);
    });

    $("#backward-command").mouseup(function() {
        base_command_end();
    });

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

    // Mouse Arrows
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

    // MUSIC
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

    // ROSLIB
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
        name : '/state_machine/voice_actions',
        messageType : 'std_msgs/String'   
    });

    listener.subscribe(function(message) {
        // alert("hello");
        console.log('Received message on ' + listener.name + ': ' + message.data);
        $("#command-list").prepend('<li class="list-group-item">' + message.data + '</li>');
        // listener.unsubscribe();
    });

    console.log(listener);

    // ROS2D (map)
    var viewer = new ROS2D.Viewer({
        divID : 'map_nav',
        width : 560,
        height : 420
    });

    nav = new NAV2D.OccupancyGridClientNav({
        ros : ros,
        rootObject : viewer.scene,
        viewer : viewer,
        serverName : '/move_base'
    });

});


