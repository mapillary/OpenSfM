
function onSyncHandler(data) {
    console.log("OnSynchandler", data);
}

function clickedMe(button, e) {
    console.log("ClickedMe", button, e);

    post_json({ event: button, data: {}});
}
function changeMe(e) {
    console.log("ClickedMe", e)
}

function initialize_view() {
    var sse = initialize_event_source("/stream", [
            { event: "sync", handler: onSyncHandler }
    ]);
    window.onunload = () => {
        sse.close();
    }

    button_bind("addGCP", clickedMe);
    button_bind("delGCP", clickedMe);

    check_bind("showgcpnames", changeMe);
    check_bind("stickyzoom", changeMe);

    button_bind("rigid", clickedMe);
    button_bind("flex", clickedMe);
    button_bind("full", clickedMe);

    button_bind("load", clickedMe);
    button_bind("save", clickedMe);
    button_bind("save_as", clickedMe);

    post_json({ event: "sync", data: {}});
}

window.onload = initialize_view;
