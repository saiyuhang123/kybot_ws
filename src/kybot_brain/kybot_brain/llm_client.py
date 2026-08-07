"""Qwen (dashscope OpenAI 兼容模式) chat + function calling 客户端.

纯 requests 实现, 无新增 pip 依赖. 与 qwen_vision 包保持同一套
DASHSCOPE_API_KEY 环境变量约定.
"""

import json

import requests


class LLMError(Exception):
    """LLM 调用失败 (网络/鉴权/限流/返回格式异常)."""


class LLMClient:
    """OpenAI 兼容 /chat/completions 的最小封装, 支持 tools."""

    def __init__(self, api_key, api_base, model, timeout_sec=60.0, logger=None):
        self._api_key = api_key
        self._url = api_base.rstrip('/') + '/chat/completions'
        self._model = model
        self._timeout = timeout_sec
        self._logger = logger

    def _log_warn(self, msg):
        if self._logger is not None:
            self._logger.warn(msg)

    def chat(self, messages, tools=None):
        """一轮对话.

        返回 assistant message dict: {'role','content','tool_calls'?}.
        失败抛 LLMError.
        """
        payload = {
            'model': self._model,
            'messages': messages,
            'temperature': 0.3,
        }
        if tools:
            payload['tools'] = tools
            payload['tool_choice'] = 'auto'
        headers = {
            'Authorization': 'Bearer ' + self._api_key,
            'Content-Type': 'application/json',
        }
        try:
            resp = requests.post(self._url, headers=headers, json=payload,
                                 timeout=self._timeout)
        except requests.RequestException as exc:
            raise LLMError('LLM 请求失败: %s' % exc)
        if resp.status_code != 200:
            raise LLMError('LLM 返回 HTTP %d: %s'
                           % (resp.status_code, resp.text[:300]))
        try:
            body = resp.json()
            message = body['choices'][0]['message']
        except (ValueError, KeyError, IndexError) as exc:
            raise LLMError('LLM 返回格式异常: %s' % exc)
        # 兼容 reasoning 模型: content 可能为 None
        if message.get('content') is None:
            message['content'] = ''
        tool_calls = message.get('tool_calls') or []
        if tool_calls:
            self._log_warn('LLM 请求调用 %d 个工具: %s'
                           % (len(tool_calls),
                              json.dumps([t.get('function', {}).get('name')
                                          for t in tool_calls],
                                         ensure_ascii=False)))
        return message
