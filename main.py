import sys
from handlers import unbag, axiswise_rot, axis_swap, conv_ply_xyz, height_color
from entities import *
from log import start_log


def unpack_query(data: dict):
    result_query = Query(operation_id=data['id'],
                         operation_type=data['operation_type'],
                         source_dir=[],
                         output_dir=data['output_path'],
                         params=data['params'])
    if result_query.operation_type == "unbag":
        result_query.source_dir = list(data['files'])
    elif result_query.operation_type in ("conv_ply_xyz", "height_color", "axis_swap", "axiswise_rot"):
        result_query.source_dir += [data['file']]
    else:
        sys.exit("Wrong operation type")
    return result_query


def exec_handler(q: Query):
    match q.operation_type:
        case "unbag":
            return unbag.start(q)
        case "conv_ply_xyz":
            return conv_ply_xyz.start(q)
        case "height_color":
            return height_color.start(q)
        case "axis_swap":
            return axis_swap.start(q)
        case "axiswise_rot":
            return axiswise_rot.start(q)


def main(f: str):
    start_log()
    query_file = open(f, "r", encoding="utf-8")
    all_data = json.load(query_file)
    input_request = all_data['Входящий request']
    expected_output_response = dict(all_data["Ожидаемый response"])
    query = unpack_query(input_request)
    handler_result = exec_handler(query)
    print("Ожидаемый ответ:\n", expected_output_response)
    print("Полученный ответ:\n", handler_result)


if __name__ == "__main__":
    file = "examples/messages/unbag.json"
    main(file)


