function getCookie(key) {
    return document.cookie
        .split("; ")
        .map((cookie) => cookie.trim())
        .find((cookie) => cookie.startsWith(`${key}=`))
        ?.split("=")[1];
}

function setCookie(key, value) {
    document.cookie = `${key}=${value}`;
}
