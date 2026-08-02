# JuPedSim XML Multi-Exit Transition Validation

Dieses Dokument beschreibt ein natives (kein Python) Validierungs-Setup fuer
`jupedsim` mit XML-Szenarien.

## Vergleichsszenarien

- `round_robin_multi_exit_validation.xml`
- `adaptive_multi_exit_validation.xml`

Beide Szenarien sind identisch (Geometrie, Agenten), unterscheiden sich nur in
`<exits mode=\"...\">`.

## XML-Elemente fuer Multi-Exit

```xml
<decision x="20.0" y="15.0" distance="1.0" />

<exits
  mode="adaptive|least_targeted|round_robin|fixed"
  fixed_index="0"
  expected_time_weight="1.0"
  density_weight="1.0"
  queue_weight="0.0"
  switch_penalty="0.0"
  decision_interval="1"
  reconsideration_threshold="0.0">
  <exit weight="1">...</exit>
  <exit weight="1">...</exit>
</exits>
```

Hinweise:

- `<decision .../>` ist Pflicht, wenn `<exits>` verwendet wird.
- `weight` wird nur bei `mode="round_robin"` ausgewertet.
- Bei `mode="fixed"` wird `fixed_index` genutzt.

## Lauf mit nativer CLI

```bash
./build/bin/jupedsim examples/xml/round_robin_multi_exit_validation.xml \
  --out-jsp /tmp/round_robin_multi_exit_validation.jsp

./build/bin/jupedsim examples/xml/adaptive_multi_exit_validation.xml \
  --out-jsp /tmp/adaptive_multi_exit_validation.jsp
```

`jupedsim` gibt am Ende `iterations=...` aus. Diese Zahl kann direkt fuer
einen A/B-Vergleich verwendet werden.

Beispiel aus diesem Repo (Stand dieser Konfiguration):

- `round_robin_multi_exit_validation.xml`: `iterations=6280`
- `adaptive_multi_exit_validation.xml`: `iterations=4927`

## Was dieser Vergleich zeigt und was nicht

Im `adaptive`-Lauf gehen **alle 120 Personen durch Ausgang 2, keine durch
Ausgang 1**. Das sieht nach einer Fehlkonfiguration von `density_weight` aus,
ist aber das korrekte Ergebnis fuer diese Geometrie:

| | Entfernung vom Entscheidungspunkt (20,15) | Breite |
| --- | --- | --- |
| Ausgang 1 | 60.4 m | 0.80 m |
| Ausgang 2 | 25.6 m | 1.00 m |

Ausgang 1 ist gleichzeitig 2.4x weiter entfernt **und** schmaler, also strikt
schlechter. Es gibt hier nichts, wozu man adaptieren koennte, und der
Algorithmus liefert mit `density_weight="0.1"` die richtige Antwort.

`density_weight` hochzudrehen erzeugt zwar eine sichtbare Aufteilung, macht das
Ergebnis aber schlechter, nicht besser:

| `density_weight` | `iterations` | Ausgang 1 | Ausgang 2 |
| --- | --- | --- | --- |
| 0.1 (aktuell) | 4927 | 0 | 120 |
| 1.0 | 4927 | 0 | 120 |
| 2.0 | 5090 | 0 | 120 |
| 3.0 | 5002 | 0 | 120 |
| 5.0 | 5169 | 10 | 110 |
| 8.0 | 5510 | 7 | 113 |
| 12.0 | 4900 | 0 | 120 |
| 20.0 | 5792 | 20 | 100 |

Jeder Wert, der aufteilt, ist langsamer als die aktuelle Einstellung. Der
Verlauf ist zudem nicht monoton (`12.0` schickt wieder alle durch einen Ausgang
und ist der schnellste Lauf ueberhaupt), d.h. kleine Parameteraenderungen
kippen das Ergebnis unvorhersehbar. Auch Ausgang 1 auf 2.0 m zu verbreitern
aendert nichts an `0/120` -- die Entfernung dominiert, nicht die Breite.

**Der A/B-Vergleich belegt daher:** freie Ausgangswahl schlaegt starres
Alternieren (4927 gegen 6280 Iterationen; `round_robin` teilt stur 60/60 auf und
schickt die Haelfte auf den 60-m-Weg).

**Er belegt nicht:** dass Stauvermeidung funktioniert. Dafuer braeuchte es ein
Szenario mit zwei annaehernd gleichwertigen Ausgaengen, in dem der Umweg sich
bei hoher Auslastung tatsaechlich lohnt.

## Neubewertung waehrend des Laufs

Seit `7fadef6f` wird die Ausgangswahl bei `mode="adaptive"` waehrend des Laufs
neu bewertet, nicht mehr nur einmal beim Passieren des Entscheidungspunkts.
Erst dadurch wirken `decision_interval` und `reconsideration_threshold`
ueberhaupt.

- `decision_interval="1"` bewertet in jedem Frame neu.
- `reconsideration_threshold` und `switch_penalty` steuern, wie stark eine
  Person an ihrer bisherigen Wahl festhaelt. Beide auf `0.0` bedeutet: kein
  Beharren.
- `decision_interval="100000"` reproduziert das fruehere Verhalten mit genau
  einer Entscheidung pro Person.

Die anderen Modi (`round_robin`, `least_targeted`, `fixed`) bewerten
unveraendert nur einmal.

## Monte-Carlo Unsicherheitsanalyse (P50/P95)

Fuer automatisierte Unsicherheitsauswertung (mehrere Seeds + KPI-Report mit
P50/P95, Exit-Throughput und Hotspots):

```bash
python3 scripts/evac_uncertainty_cli.py \
  --scenario examples/xml/adaptive_multi_exit_validation.xml \
  --cli build/bin/jupedsim \
  --runs 20 \
  --output-dir artifacts/evac_uncertainty
```

Erzeugte Reports:

- `artifacts/evac_uncertainty/summary.json`
- `artifacts/evac_uncertainty/summary.md`
