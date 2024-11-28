"""Utility to visualize photo plans.
"""

import typing as T

import plotly.graph_objects as go

from src.data_model import Waypoint


def plot_photo_plan(photo_plans: T.List[Waypoint], max_speed, finish_time) -> go.Figure:
    """Plot the photo plan on a 2D grid.

    Args:
        photo_plans (T.List[Waypoint]): List of waypoints for the photo plan.

    Returns:
        T.Any: Plotly figure object.
    """
    x_coords = [point.X for point in photo_plans]
    y_coords = [point.Y for point in photo_plans]
    
    # Create the scatter plot
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=x_coords, y=y_coords, mode='markers+lines', name='Flight Path'))
    
    # Add titles and labels
    fig.update_layout(
        title="Flight Plan Visualization",
        xaxis_title="X Coordinate",
        yaxis_title="Y Coordinate",
        showlegend=True
    )

    fig.add_annotation(
        x=1.08,  
        y=1.2,
        text=f"Time to complete this plan: {int(finish_time)} seconds <br>Achievable speed in this plan: {int(max_speed)} m/s",
        showarrow=False,
        font=dict(size=14, color="blue"),
        xref="paper",
        yref="paper",
        align="left"
    )
    
    
    return fig
