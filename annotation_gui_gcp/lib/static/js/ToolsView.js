let pointListBox;
let currentPointID;
window.addEventListener('DOMContentLoaded', onDOMLoaded);

function onSyncHandler(data) {
    currentPointID = data["selected_point"];
    populatePointsList(data["points"]);
}

function populatePointsList(points) {
    const L = pointListBox.options.length - 1;
    for (let i = L; i >= 0; i--) {
        pointListBox.remove(i);
    }

    for (let point_id in points) {
        const opt = document.createElement("option");
        opt.text = "Point " + point_id;
        opt.value = point_id;

        if (opt.value == currentPointID) {
            opt.style.fontWeight = "bold";
        }

        pointListBox.options.add(opt);
    }
    pointListBox.value = currentPointID;
}

function clickedMe(button, e) {
    post_json({ event: button, data: {} });
}

function onPointSelect() {
    const opt = pointListBox.options[pointListBox.options.selectedIndex];
    const point_id = opt.value;
    console.log("Selecting", point_id)
    post_json({ event: "select_cp", point_id: point_id })
}

function onDOMLoaded() {
    pointListBox = document.getElementById("pointListBox");
    pointListBox.addEventListener('change', onPointSelect);
    window.addEventListener('load', initialize_view);
}

function initialize_view() {
    const sse = initialize_event_source([
        { event: "sync", handler: onSyncHandler }
    ]);
    window.onunload = () => {
        sse.close();
    }

    button_bind("addCP", clickedMe);
    button_bind("delCP", clickedMe);

    // check_bind("showcpnames", changeMe);
    // check_bind("stickyzoom", changeMe);

    button_bind("rigid", clickedMe);
    button_bind("flex", clickedMe);
    button_bind("full", clickedMe);

    button_bind("load", clickedMe);
    button_bind("save", clickedMe);
    // button_bind("save_as", clickedMe);

    post_json({ event: "init" });
}
