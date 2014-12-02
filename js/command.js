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

    // // Mouse Arrows
    // $(document).keypress(function(e){
    //     if (e.keyCode == 38) { 
    //         var params = {type:"base", x:0.1, y:0, z:0};
    //         base_interval = setInterval(function() {
    //             $.get("/", params); 
    //             }, 500
    //         );
    //         return false;
    //     }
    // });

    // $(document).keypress(function(e){
    //     if (e.keyCode == 37) { 
    //         var params = {type:"base", x:0, y:0, z:0.3};
    //         base_interval = setInterval(function() {
    //             $.get("/", params); 
    //             }, 500
    //         );
    //         return false;
    //     }
    // });

    // $(document).keypress(function(e){
    //     if (e.keyCode == 39) { 
    //         var params = {type:"base", x:0, y:0, z:-0.3};
    //         base_interval = setInterval(function() {
    //             $.get("/", params); 
    //             }, 500
    //         );
    //         return false;
    //     }
    // });

    // $(document).keypress(function(e){
    //     if (e.keyCode == 40) { 
    //         var params = {type:"base", x:-0.1, y:0, z:0};
    //         base_interval = setInterval(function() {
    //             $.get("/", params); 
    //             }, 500
    //         );
    //         return false;
    //     }
    // });

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

    var nav = new NAV2D.OccupancyGridClientNav({
        ros : ros,
        rootObject : viewer.scene,
        viewer : viewer,
        serverName : '/move_base'
    });

























    // MUSIC PLAYER

    'use strict';
    $('audio[controls]').before(function(){
        $( document ).tooltip();

        var colors = [
                        "Bonobo - Cirrus.wav",
                        "Bonobo - Recurring.wav",
                        "Bonobo - Silver.wav",
                        "Rick Astley - Never Gonna Give You Up.wav",
                        "Sir MixALot - Baby Got Back.wav",
                        "Uppermost - Beautiful Light.wav",
                        "Uppermost - Energy.wav"
                    ];
         
        $('#searchMusic').typeahead({
            source: colors,
            hint: true,
            highlight: true,
            minLength: 0
        });

        $("#searchMusic").on('focus', $("#searchMusic").typeahead.bind($("#searchMusic"), 'lookup'));

        var songMuted = new Boolean(false);
        var songOldVolume = 0.0;
        var songTotalLength = 800;
        var songCurrentTime = 0;
        var songVolume = 0.0;
        var setPlayFunc;
        var setSeekFunc;
        var showTimeFunc;
        var setVolumeFunc;
        var setMuteFunc;
        var setArtistFunc;
        var setTitleFunc;

        var controller_state_listener = new ROSLIB.Topic({
            ros : ros,
            name : '/music_player_state',
            messageType : 'dream_machine/MusicState'   
        });

        controller_state_listener.subscribe(function(message) {
            songTotalLength = message.total_length;
            setPlayFunc(message.is_playing);
            setSeekFunc(message.current_time);

            showTimeFunc();
            setVolumeFunc(message.volume);               
            setMuteFunc(); // Mute after setting the volume
        });


         var music_commands_pub = new ROSLIB.Topic({
           ros : ros,
           name : '/music_commands',
           messageType : 'dream_machine/MusicCommand'
         });

        function pub_play() {
            var play_msg = new ROSLIB.Message({
              command : "play"
            });

            music_commands_pub.publish(play_msg);
         };

        function pub_pause() {
            var pause_msg = new ROSLIB.Message({
              command : "pause"
            });

            music_commands_pub.publish(pause_msg);
         };

        //  function stop() {
        //     music_commands_pub.publish(
        //         new ROSLIB.MESSAGE({
        //             command : "stop",
        //             args : { }
        //     }));
        //  }

        function pub_seek_to(val) {
            var seek_to_msg = new ROSLIB.Message({
              command : "seek_to",
              args : [val.toString()]
            });

            music_commands_pub.publish(seek_to_msg);
         }

         function pub_set_volume(val) {
            var set_volume_msg = new ROSLIB.Message({
              command : "set_volume",
              args : [val.toString()]
            });

            music_commands_pub.publish(set_volume_msg);
         }

        function pub_load_song(val) {
            var load_song_msg = new ROSLIB.Message({
              command : "load",
              args : [val]
            });

            music_commands_pub.publish(load_song_msg);
         };


        var song = this;
            song.controls=false;
        var player_box = document.createElement('div');
            $(player_box).addClass($(song).attr('class') + ' well container-fluid playa');
            $(player_box).attr('id', "player_control");
        var data_sec = document.createElement('section');
            $(data_sec).addClass('collapse');
        var toggle_holder = document.createElement('div');
            $(toggle_holder).addClass('btn-group row-fluid');
        var data_toggle = document.createElement('a');
            $(data_toggle).html('<i class="icon-reorder"></i>');
            $(data_toggle).addClass('btn btn-block');
            $(data_toggle).attr('style', 'opacity:0.3');
            $(data_toggle).click(function (){$(data_sec).collapse('toggle');});
            $(data_toggle).attr('title', 'Details');
            $(data_toggle).tooltip({'container': 'body', 'placement': 'top', 'html': true});
            $(toggle_holder).append(data_toggle);
        var data_table = document.createElement('table');
            $(data_table).addClass('table table-condensed');
            $(data_table).attr('id', 'data_table');
        var player = document.createElement('section');
            $(player).addClass('btn-group row-fluid');
        var load_error = function(){
            console.log('error');
            $(player_box).find('.btn').addClass('disabled');
            $(player_box).find('input[type="range"]').hide();
            $(player_box).find('.icon-spin').text('Error');
            $(player_box).find('.icon-spin').parent().attr('title', 'There was an error loading the audio.');
            $(player_box).find('.icon-spin').parent().tooltip('fixTitle');
            $(player_box).find('.icon-spin').removeClass('icon-spinner icon-spin');
        };
        var addPlay = function() {
            var play = document.createElement('button');
            $(play).addClass('btn span1 playBtn play');
            $(play).html('<i class="icon-play"></i>');
            
            $(play).click(function() {
                if ($(play).hasClass('play')) {
                    $(play).removeClass('play')
                    $(play).html('<i class="icon-pause"></i>');
                    // alert('play');
                    pub_play();
                } else {
                    $(play).addClass('play')
                    $(play).html('<i class="icon-play"></i>');

                    pub_pause();
                }
            });

            setPlayFunc = function(setPlay) {
                if (setPlay) {
                    $(play).removeClass('play')
                    $(play).html('<i class="icon-pause"></i>');
                } else {
                    $(play).addClass('play')
                    $(play).html('<i class="icon-play"></i>');

                    //pause();
                }
            }
            $(player).append(play);
        };
        var addSeek = function() {
            var seek = document.createElement('input');
                $(seek).attr({
                    'id' : 'specialVal',
                    'type': 'range',
                    'min': 0,
                    'value': 0,
                    'class': 'seek'
                });
            seek.progress = function () {
                var bg = 'rgba(223, 240, 216, 1) 0%';
                bg += ', rgba(223, 240, 216, 1) ' + ((songCurrentTime/songTotalLength) * 100) + '%';
                bg += ', rgba(223, 240, 216, 0) ' + ((songCurrentTime/songTotalLength) * 100) + '%';
                for (var i=0; i<song.buffered.length; i++){
                    if (song.buffered.end(i) > songCurrentTime && isNaN(song.buffered.end(i)) === false && isNaN(song.buffered.start(i)) === false){
                        var bufferedstart;
                        var bufferedend;
                        if (song.buffered.end(i) < songTotalLength) {
                            bufferedend = ((song.buffered.end(i)/songTotalLength) * 100);
                        }
                        else {
                            bufferedend = 100;
                        }
                        if (song.buffered.start(i) > songCurrentTime){
                            bufferedstart = ((song.buffered.start(i)/songTotalLength) * 100);
                        }
                        else {
                            bufferedstart = ((songCurrentTime/songTotalLength) * 100);
                        }
                        bg += ', rgba(217, 237, 247, 0) ' + bufferedstart + '%';
                        bg += ', rgba(217, 237, 247, 1) ' + bufferedstart + '%';
                        bg += ', rgba(217, 237, 247, 1) ' + bufferedend + '%';
                        bg += ', rgba(217, 237, 247, 0) ' + bufferedend + '%';
                    }                       
                }
                $(seek).css('background', '-webkit-linear-gradient(left, ' + bg + ')');
                //These may be re-enabled when/if other browsers support the background like webkit
                //$(seek).css('background','-o-linear-gradient(left,  ' + bg + ')');
                //$(seek).css('background','-moz-linear-gradient(left,  ' + bg + ')');
                //$(seek).css('background','-ms-linear-gradient(left,  ' + bg + ')');
                //$(seek).css('background','linear-gradient(to right,  ' + bg + ')');
                $(seek).css('background-color', '#ddd');
            };
            seek.set = function () {
                $(seek).val(songCurrentTime);
                seek.progress();
            };

            setSeekFunc = function (val) {
                $(seek).attr({
                    'max': songTotalLength,
                    'step': songTotalLength / 100
                });

                songCurrentTime = parseInt(val);
                $(seek).val(songCurrentTime);
                seek.progress();
            };

            seek.slide = function () {
                songCurrentTime = parseInt($(seek).val());

                pub_seek_to(songCurrentTime);

                seek.progress();
            };
            seek.init = function () {
                $(seek).attr({
                    'max': songTotalLength,
                    'step': songTotalLength / 100
                });
                seek.set();
            };
            seek.reset = function () {
                $(seek).val(0);
                songCurrentTime = parseInt($(seek).val());
                //pub_seek_to(songCurrentTime);
                if(!song.loop)
                {
                    // pause();
                } else {
                    // play();
                }
            };
            var seek_wrapper = document.createElement('div');
                $(seek_wrapper).addClass('btn span4');
                $(seek_wrapper).attr('id', 'seeker1');

            $(seek_wrapper).append(seek);
            $(seek).on('change', seek.slide);

            seek.init();

            $(player).append(seek_wrapper);
        };
        var addTime = function() {
            var time = document.createElement('a');
                $(time).addClass('btn span3');
                $(time).tooltip({'container': 'body', 'placement': 'right', 'html': true});
            time.twodigit = function (myNum) {
                return ("0" + myNum).slice(-2);
            };
            time.timesplit = function (a) {
                if (isNaN(a)){return '<i class="icon-spinner icon-spin"></i>';}
                var hours = Math.floor(a / 3600);
                var minutes = Math.floor(a / 60) - (hours * 60);
                var seconds = Math.floor(a) - (hours * 3600) - (minutes * 60);
                var timeStr = time.twodigit(minutes) + ':' + time.twodigit(seconds);
                if (hours > 0) {
                    timeStr = hours + ':' + timeStr;
                }
                return timeStr;
            };
            showTimeFunc = time.showtime = function () {
                $(time).html((time.timesplit(songCurrentTime)) + ' / ' + time.timesplit(songTotalLength));
                $(time).attr({'title': 'Click to Reset<hr style="padding:0; margin:0;" />Position: ' + (time.timesplit(songCurrentTime))});
                if (!song.paused){
                    $(time).html(time.timesplit(songCurrentTime));
                    $(time).attr({'title': 'Click to Reset<hr style="padding:0; margin:0;" />Length: ' + (time.timesplit(songTotalLength))});
                }
                $(time).tooltip('fixTitle');
            };
            $(time).click(function () {
                songCurrentTime = 0;

                pub_pause();
                pub_seek_to(0);

                time.showtime()


                $(time).tooltip('fixTitle');
                $(time).tooltip('show');
            });

            time.showtime()

            $(player).append(time);
        };
        var addMute = function() {
            var mute = document.createElement('button');
                $(mute).addClass('btn span1');
            mute.checkVolume = function () {
                if (songVolume > 0.5 && !songMuted) {
                    $(mute).html('<i class="icon-volume-up"></i>');
                } else if (songVolume < 0.5 && songVolume > 0 && !songMuted) {
                    $(mute).html('<i class="icon-volume-down"></i>');
                } else {
                    $(mute).html('<i class="icon-volume-off"></i>');
                }
            };
            $(mute).click(function () {
                if (songVolume == 0.0) {
                    songVolume = songOldVolume;
                } else {
                    songOldVolume = songVolume;
                    songVolume = 0.0;
                }
                pub_set_volume(songVolume);
                mute.checkVolume();
            });

            setMuteFunc = function() {
                console.log(parseFloat(songVolume));
                if (songVolume > 0.5) {
                    $(mute).html('<i class="icon-volume-up"></i>');
                } else if (songVolume < 0.5 && songVolume > 0) {
                    $(mute).html('<i class="icon-volume-down"></i>');
                } else {
                    $(mute).html('<i class="icon-volume-off"></i>');
                }
            };
            mute.checkVolume();
            $(song).on('volumechange', mute.checkVolume);
            $(player).append(mute);
        };
        var addVolume = function() {
            var volume = document.createElement('input');
                $(volume).attr({
                    'type': 'range',
                    'min': 0,
                    'max': 1,
                    'step': 1 / 100,
                    'value': 1
                });
            volume.slide = function () {
                songMuted = false;
                songVolume = parseFloat($(volume).val());
                pub_set_volume(songVolume);
            };

            setVolumeFunc = function(val) {
                songMuted = false;
                $(volume).val(val);
                songVolume = val;
            }
            volume.set = function () {
                $(volume).val(songVolume);
            };
            var vol_wrapper = document.createElement('div');
                $(vol_wrapper).addClass('btn span3');
                $(vol_wrapper).attr('id', 'seeker2');
            $(vol_wrapper).append(volume);
            $(volume).on("change", volume.slide);
            $(song).on('volumechange', volume.set);
            $(player).append(vol_wrapper);
        };
        var addAlbumArt = function() {
            var albumArt = document.createElement('img');
                $(albumArt).addClass('thumbnail');
                $(albumArt).attr('src', $(song).data('infoAlbumArt'));
            $(data_sec).append(albumArt);
        };
        var addInfo = function(title, dataId) {
            var row = document.createElement('tr');
            var head = document.createElement('th');
            var data = document.createElement('td');
            $(head).html(title);
            $(data).html($(song).data(dataId));
            $(row).append(head);
            $(row).append(data);
            $(data_table).append(row);

            return function(val) {
                $(data).html(val);
            };
        };
        var addData = function() {
            setArtistFunc = addInfo('Artist', 'infoArtist');
            setTitleFunc = addInfo('Title', 'infoTitle');

            if ($(data_table).html() !== ""){
                $(data_sec).append(data_table);
                $(player_box).append(toggle_holder);
                $(player_box).append(data_sec);
            }
        };
        var addPlayer = function() {
            addPlay();
            addSeek();
            addTime();
            addMute();
            addVolume();

            $(player_box).append(player);
        };
        var addAttribution = function() {
            var attribution = document.createElement('small');
                $(attribution).addClass('pull-right muted');
            if (typeof($(song).data('infoAttLink')) !== 'undefined'){
                var attribution_link = document.createElement('a');
                    $(attribution_link).addClass('muted');
                    $(attribution_link).attr('href', $(song).data('infoAttLink'));
                    $(attribution_link).html($(song).data('infoAtt'));
                $(attribution).append(attribution_link);
            }
            else {
                $(attribution).html($(song).data('infoAtt'));
            }
            $(player_box).append(attribution);
        };
        var fillPlayerBox = function() {
            addData();
            addPlayer();
            if (typeof($(song).data('infoAtt')) !== 'undefined'){ addAttribution();}
        };
        fillPlayerBox();
        $(song).on('error', function(){
            load_error();
        });


        $(".load").click(function() {
            var splitSongName = $("#searchMusic").val().split('-');
            if(splitSongName.length == 2)
            {
                var artist = splitSongName[0];
                var splitSong = splitSongName[1].split('.');
                var title = splitSong[0];

                setArtistFunc(artist);
                setTitleFunc(title);
            }

            pub_load_song($("#searchMusic").val());
        });


        function chooseandLoadRandomSong() {
            $("#searchMusic").val(colors[Math.floor(Math.random()*(colors.length))]);
            $(".load")[0].click();
        };

        // setInterval( 
        //     function() {
        //         setPlayFunc(Math.random() >= 0.5);
        //         setSeekFunc(Math.floor(Math.random()*(songTotalLength+1)));
        //         showTimeFunc();
        //         setVolumeFunc(Math.random());               
        //         setMuteFunc(); // Mute after setting the volume
        //     }, 1000);

        return player_box;
    });

});


