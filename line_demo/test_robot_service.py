"""服务切换的失败分支测试：所有 RPC 使用替身，不初始化 DDS 或连接机器人。"""
import io
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import robot_service as service


def state(status=0, protect=0):
    return SimpleNamespace(name="example_arm", status=status, protect=protect)


def entries(target, running_code=0):
    return [SimpleNamespace(name="robot_state", status=running_code, protect=1), target]


class TestRobotService(unittest.TestCase):
    def test_list_is_read_only(self):
        client = Mock()
        client.ServiceList.return_value = (0, [state()])
        self.assertEqual(service.service_list(client)[0].name, "example_arm")
        client.ServiceSwitch.assert_not_called()

    def test_missing_or_protected_service_is_not_switched(self):
        for services in ([], entries(state(protect=1))):
            client = Mock()
            client.ServiceList.return_value = (0, services)
            with self.assertRaises(RuntimeError):
                service.switch_service(client, "example_arm", False)
            client.ServiceSwitch.assert_not_called()

    def test_already_stopped_does_not_toggle(self):
        client = Mock()
        client.ServiceList.return_value = (0, entries(state(1)))
        self.assertEqual(service.switch_service(client, "example_arm", False).status, 1)
        client.ServiceSwitch.assert_not_called()

    def test_stop_and_restore_confirm_readback(self):
        for enabled in (False, True):
            client = Mock()
            client.ServiceList.side_effect = [(0, entries(state(int(enabled)))),
                                              (0, entries(state(int(not enabled))))]
            client.ServiceSwitch.return_value = 0
            result = service.switch_service(client, "example_arm", enabled)
            client.ServiceSwitch.assert_called_once_with("example_arm", enabled)
            self.assertEqual(result.status, int(not enabled))
            self.assertEqual(client.ServiceList.call_count, 2)

    def test_rpc_failure_or_unconfirmed_switch_is_not_success(self):
        for code in (5202, 5201, 3102, 0):
            client = Mock()
            client.ServiceList.return_value = (0, entries(state()))
            client.ServiceSwitch.return_value = code
            with self.assertRaises(RuntimeError):
                service.switch_service(client, "example_arm", False, timeout=0)
            client.ServiceSwitch.assert_called_once()

    def test_user_g1d_list_does_not_invert_running_and_stopped(self):
        # 现场列表：robot_state=0 且 RPC 正常响应，ai_sport=1。
        sample = json.loads((Path(__file__).resolve().parents[1] /
                             "references/g1d_service_inventory_sample.json").read_text())
        services = [SimpleNamespace(**entry) for entry in sample["services"]]
        self.assertEqual(service.status_codes(services), (0, 1))
        self.assertNotIn("g1_arm_example", [s.name for s in services])
        client = Mock()
        client.ServiceList.return_value = (0, services)
        result = service.switch_service(client, "ai_sport", False)
        self.assertEqual(result.status, 1)
        client.ServiceSwitch.assert_not_called()
        # 同样的当前状态绝不能把 on 当作已运行而跳过；它必须发送 True 并读回 0。
        client.ServiceList.side_effect = [(0, entries(state(1))), (0, entries(state(0)))]
        client.ServiceSwitch.return_value = 0
        self.assertEqual(service.switch_service(client, "example_arm", True).status, 0)
        client.ServiceSwitch.assert_called_once_with("example_arm", True)

    def test_missing_anchor_and_unknown_codes_cannot_switch(self):
        for services in ([state()], entries(state(0), running_code=3)):
            client = Mock()
            client.ServiceList.return_value = (0, services)
            with self.assertRaises(RuntimeError):
                service.switch_service(client, "example_arm", False)
            client.ServiceSwitch.assert_not_called()

    def test_reverse_firmware_codes_use_anchor_not_switch_boolean(self):
        client = Mock()
        client.ServiceList.side_effect = [(0, entries(state(1), running_code=1)),
                                          (0, entries(state(0), running_code=1))]
        client.ServiceSwitch.return_value = 0
        self.assertEqual(service.switch_service(client, "example_arm", False).status, 0)
        client.ServiceSwitch.assert_called_once_with("example_arm", False)

    def test_explicit_service_name_required_before_dds(self):
        with patch.object(service, "initialize") as initialize, \
                patch("sys.stderr", new=io.StringIO()):
            with self.assertRaises(SystemExit):
                service.main(["off", "--network", "eth0"])
            initialize.assert_not_called()


if __name__ == "__main__":
    unittest.main()
