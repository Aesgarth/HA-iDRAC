# Generating a Home Assistant Dashboard for HA-iDRAC

[`tools/make_dashboard.py`](../tools/make_dashboard.py) generates a complete Home Assistant dashboard (YAML) for your iDRAC server(s) from your actual entity list.

## What You Get

The generated dashboard creates a view per server with four sections (**Status**, **Temperatures**, **Cooling and power**, and **Hardware health**) built **only from Home Assistant's built-in cards** (`sections`, `grid`, `heading`, `tile`, `gauge`, `history-graph`, and `entities`), so no HACS or custom cards are needed.

## Step 1: List Your Entities

In Home Assistant, open **Developer Tools > Template**, paste the following template, and copy the output into a text file named `entities.txt`:

```jinja2
{% for s in states if 'idrac' in s.entity_id %}{{ s.entity_id }}
{% endfor %}
```

## Step 2: Run the Generator

Run [`tools/make_dashboard.py`](../tools/make_dashboard.py) on `entities.txt`:

```bash
python tools/make_dashboard.py entities.txt --shutdown --server=idrac_<alias> > idrac_dashboard.yaml
```

* `--server=idrac_<alias>` (optional, repeatable): Limits the generated dashboard to the specified server prefix (for example `--server=idrac_r720_test`), which also skips leftover entities from old installs. Omit `--server` to generate a tab for every detected iDRAC server.
* `--shutdown` (optional): Adds a **Shutdown** tile to the **Status** section with a `"Shut down this server?"` confirmation prompt.

## Step 3: Import into Home Assistant

1. In Home Assistant, go to **Settings > Dashboards > Add dashboard > New dashboard from scratch**.
2. Open the new dashboard, click the three-dot menu (**⋮**) in the top right > **Edit**, then click the three-dot menu (**⋮**) again > **Raw configuration editor**.
3. Paste the contents of `idrac_dashboard.yaml` and click **Save**.

## Tips

* **Re-run after changes:** Re-run the generator after adding new servers or after an add-on update that introduces new entities (such as per-PSU voltage/current, PSU/fan redundancy, or `last_successful_poll`).
* **Only existing entities are included:** The script inspects `entities.txt` and only adds cards/rows for entities that actually exist on your system.
* **Customizing gauge thresholds:** The default gauge severity thresholds (Hottest CPU `60` / `75` °C, Inlet `28` / `35` °C, Exhaust `45` / `55` °C for yellow/red) are starting points that you can edit directly in the generated YAML or in the Home Assistant UI editor.
