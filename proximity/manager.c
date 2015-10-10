/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2011  Nokia Corporation
 *  Copyright (C) 2011  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <glib.h>
#include <gdbus.h>
#include <bluetooth/uuid.h>

#include "adapter.h"
#include "device.h"
#include "att.h"
#include "gattrib.h"
#include "gatt.h"
#include "monitor.h"
#include "reporter.h"
#include "manager.h"

static DBusConnection *connection = NULL;

static struct enabled enabled  = {
	.linkloss = TRUE,
	.pathloss = TRUE,
	.findme = TRUE,
};

static gint primary_uuid_cmp(gconstpointer a, gconstpointer b)
{
	const struct gatt_primary *prim = a;
	const char *uuid = b;

	return g_strcmp0(prim->uuid, uuid);
}

static int attio_device_probe(struct btd_device *device, GSList *uuids)
{
	struct gatt_primary *linkloss, *txpower, *immediate;
	GSList *l, *primaries;

	primaries = btd_device_get_primaries(device);

	l = g_slist_find_custom(primaries, IMMEDIATE_ALERT_UUID,
			primary_uuid_cmp);
	immediate = (l ? l->data : NULL);

	l = g_slist_find_custom(primaries, TX_POWER_UUID, primary_uuid_cmp);
	txpower = (l ? l->data : NULL);

	l = g_slist_find_custom(primaries, LINK_LOSS_UUID, primary_uuid_cmp);
	linkloss = (l ? l->data : NULL);

	return monitor_register(connection, device, linkloss, txpower,
							immediate, &enabled);
}

static void attio_device_remove(struct btd_device *device)
{
	monitor_unregister(connection, device);
}

static struct btd_device_driver monitor_driver = {
	.name = "Proximity GATT Monitor Driver",
	.uuids = BTD_UUIDS(IMMEDIATE_ALERT_UUID, LINK_LOSS_UUID, TX_POWER_UUID),
	.probe = attio_device_probe,
	.remove = attio_device_remove,
};

static struct btd_adapter_driver reporter_server_driver = {
	.name = "Proximity GATT Reporter Driver",
	.probe = reporter_init,
	.remove = reporter_exit,
};

static void load_config_file(GKeyFile *config)
{
	char **list;
	int i;

	if (config == NULL)
		return;

	list = g_key_file_get_string_list(config, "General", "Disable",
								NULL, NULL);
	for (i = 0; list && list[i] != NULL; i++) {
		if (g_str_equal(list[i], "FindMe"))
			enabled.findme = FALSE;
		else if (g_str_equal(list[i], "LinkLoss"))
			enabled.linkloss = FALSE;
		else if (g_str_equal(list[i], "PathLoss"))
			enabled.pathloss = FALSE;
	}

	g_strfreev(list);
}

int proximity_manager_init(DBusConnection *conn, GKeyFile *config)
{
	int ret;

	load_config_file(config);

	connection = dbus_connection_ref(conn);

	ret = btd_register_device_driver(&monitor_driver);
	if (ret < 0)
		goto fail_monitor;

	ret = btd_register_adapter_driver(&reporter_server_driver);
	if (ret < 0)
		goto fail_reporter;

	return 0;

fail_reporter:
	btd_unregister_device_driver(&monitor_driver);

fail_monitor:
	dbus_connection_unref(connection);
	return ret;
}

void proximity_manager_exit(void)
{
	btd_unregister_device_driver(&monitor_driver);
	btd_unregister_adapter_driver(&reporter_server_driver);
	dbus_connection_unref(connection);
}
