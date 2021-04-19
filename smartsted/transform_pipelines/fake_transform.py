def fake_transform(coords_input):
    coords_output = coords_input[:,0]
    coords_output = ((coords_output - 250)/100).astype(float)

    return (coords_output[0], coords_output[1], 0.0)