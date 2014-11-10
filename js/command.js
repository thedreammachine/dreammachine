base_interval = null

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

$("#play-music").click(function() {
    var params = {type:"music", play:"play"};
    base_interval = setInterval(function() {
        $.get("/", params); 
        }, 10000
    );
});

$("#stop-music").click(function() {
    clearInterval(base_interval);
});