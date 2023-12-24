Na základě ukázkových kódů BT_SPP_INITIATOR a BT_SPP_ACCEPTOR jsem vytvořil kód pro spojení dvou ESP32 desek bez nutnosti zádávat párovací PIN kód a celkově jsme se snažil upravit kód do jednoušší použití pro m§j budoucí projekt. Aktuálně by kód pro spojení obou ESP32 desek měl poslat zprávu z MASTER na SLAVE a ze SLAVE na MASTER. Tyto zprávy by měly být vypsány do terminálu.

1. Problémy. po přijetí zpráv přejdou desky do úsporeného režimu a spojení není dále udržováno
2. Podivné chování MASTER při pokusech pro reconnect
