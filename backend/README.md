# sim4wis (backend)

Python 仿真后端。FastAPI + WebSocket + NumPy。

## 开发

```bash
pip install -e ".[dev]"
uvicorn sim4wis.main:app --reload --port 8010    # 8000 留给本地 LLM 推理服务
```

打开 http://localhost:8010/docs 看 API。

## 测试

```bash
pytest
```

## 包结构

```
src/sim4wis/
├── main.py            FastAPI 应用入口
├── api/               REST + WebSocket 端点
├── core/              仿真主循环、状态总线
├── vehicle/           车辆模型（VehicleModel 抽象基类 + 各实现）
├── controller/        控制策略（ControllerStrategy 基类 + 五种内置）
├── environment/       路面与扰动
├── input/             输入适配（键盘、未来 USB）
├── project/           YAML 项目文件读写
└── recorder/          数据记录与导出
```
