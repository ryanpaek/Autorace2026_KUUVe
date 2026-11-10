"""확인된 명령을 FSM의 상태 입력으로 전달한다. 메시지 수신으로 미션을 재진입하지 않는다."""

COMMAND_CLASSES = {
    'PAUSE_UNTIL_GREEN': 'red', 'RESUME_NORMAL': 'green',
    'SLOW': 'slow', 'CROSS_BUMP_SLOWLY': 'bump',
    'STOP_AT_PEDESTRIAN_LINE': 'pedestrian', 'START_PARKING': 'parking',
}


class CommandInbox:
    def __init__(self):
        self.mode = None
        self.classes = {}

    def set_mode(self, mode):
        if self.mode != mode:
            self.mode = mode
            self.classes = {}

    def receive(self, data, mode):
        self.set_mode(mode)
        if not isinstance(data, dict) or data.get('mode') != mode:
            return False
        commands = data.get('commands')
        if data.get('schema_version') != 2 or not isinstance(commands, list):
            return False
        classes = {}
        for command in commands:
            if not isinstance(command, dict) or not isinstance(command.get('request_id'), str):
                return False
            if not command['request_id']:
                return False
            name = command.get('command')
            if name == 'SELECT_ROUTE':
                cls = {'A': 'fork_a', 'B': 'fork_b'}.get(command.get('route'))
            else:
                cls = COMMAND_CLASSES.get(name)
            if cls:
                classes[cls] = 1.0
        # 지속 상태 입력. new_request 재전송·새 ID로도 FSM 실행 상태를 초기화하지 않는다.
        # 첫 메시지를 놓쳐도 유지 메시지부터 수신해 시작할 수 있다.
        self.classes = classes
        return True
